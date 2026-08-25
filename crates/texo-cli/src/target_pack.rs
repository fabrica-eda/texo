//! Installable, checksummed ECP5 runtime target packs.

use std::env;
use std::error::Error;
use std::fmt;
use std::fs::{self, File};
use std::io::{self, BufReader, Read, Write};
use std::path::{Component, Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::Deserialize;
use sha2::{Digest, Sha256};

const MAGIC: &[u8] = b"TEXO_TARGET_PACK\n";
const FORMAT_VERSION: u32 = 1;
const VERIFIED_MARKER: &str = ".texo-verified";
const CATALOG: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../architectures/ecp5/catalog.json"
));

/// Paths supplied by one installed ECP5 target pack.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ecp5TargetPack {
    /// Pack installation root.
    pub root: PathBuf,
    /// Expanded Texo routing/timing architecture.
    pub architecture: PathBuf,
    /// Decompressed empty-device Project Trellis configuration.
    pub base_config: PathBuf,
    /// Project Trellis IO-bank metadata.
    pub iodb: PathBuf,
    /// Minimal Project Trellis database used by the bundled codec.
    pub database: PathBuf,
    /// Pack-local ECP5 bitstream codec.
    pub ecppack: PathBuf,
}

impl Ecp5TargetPack {
    /// Opens and validates the required layout of an installed pack.
    ///
    /// # Errors
    ///
    /// Returns an error for an incompatible platform/format or a missing,
    /// truncated, or checksum-invalid architecture/runtime file.
    pub fn open(root: impl Into<PathBuf>) -> Result<Self, TargetPackError> {
        let root = root.into();
        let manifest: PackManifest =
            serde_json::from_reader(BufReader::new(File::open(root.join("manifest.json"))?))?;
        if manifest.format_version != FORMAT_VERSION {
            return Err(TargetPackError::new(format!(
                "unsupported installed target-pack version {}",
                manifest.format_version
            )));
        }
        if manifest.platform != host_platform()? {
            return Err(TargetPackError::new(format!(
                "target pack platform {} does not match {}",
                manifest.platform,
                host_platform()?
            )));
        }
        let architecture = root.join("architecture.txdb");
        verify_installed_architecture(&root, &architecture, &manifest.architecture)?;
        let pack = Self {
            architecture,
            base_config: root.join("base.config"),
            iodb: root.join("iodb.json"),
            database: root.join("database"),
            ecppack: root.join("bin/ecppack"),
            root,
        };
        for required in [
            &pack.base_config,
            &pack.iodb,
            &pack.database.join("devices.json"),
            &pack.ecppack,
        ] {
            if !required.is_file() {
                return Err(TargetPackError::new(format!(
                    "target pack is incomplete: {}",
                    required.display()
                )));
            }
        }
        Ok(pack)
    }

    /// Target device recorded by the pack manifest.
    ///
    /// # Errors
    ///
    /// Returns an error if the installed manifest cannot be read or decoded.
    pub fn device(&self) -> Result<String, TargetPackError> {
        let manifest: PackManifest =
            serde_json::from_reader(BufReader::new(File::open(self.root.join("manifest.json"))?))?;
        Ok(manifest.device)
    }
}

/// Installs a local `.txpkg.zst` into the Texo target cache.
///
/// # Errors
///
/// Returns an error when the archive is malformed, unsafe, checksum-invalid,
/// incompatible with this host, or cannot be written to the target cache.
pub fn install_ecp5_target_pack(
    archive: &Path,
    cache_root: Option<&Path>,
) -> Result<Ecp5TargetPack, TargetPackError> {
    verify_published_archive(archive)?;
    install_verified_target_pack(archive, cache_root)
}

fn install_verified_target_pack(
    archive: &Path,
    cache_root: Option<&Path>,
) -> Result<Ecp5TargetPack, TargetPackError> {
    let cache_root = match cache_root {
        Some(path) => path.to_path_buf(),
        None => target_cache_root()?,
    };
    fs::create_dir_all(&cache_root)?;
    let temporary = cache_root.join(format!(
        ".install-{}-{}",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_err(|error| TargetPackError::new(error.to_string()))?
            .as_nanos()
    ));
    fs::create_dir(&temporary)?;
    let result = unpack_archive(archive, &temporary).and_then(|()| {
        let manifest: PackManifest =
            serde_json::from_reader(BufReader::new(File::open(temporary.join("manifest.json"))?))?;
        let destination = cache_root.join(format!(
            "{}-{}-{}",
            manifest.device, manifest.artifact_stem, manifest.platform
        ));
        if destination.exists() {
            let installed = Ecp5TargetPack::open(&destination)?;
            if installed.device()? != manifest.device {
                return Err(TargetPackError::new(format!(
                    "installed target at {} has conflicting metadata",
                    destination.display()
                )));
            }
            fs::remove_dir_all(&temporary)?;
            return Ok(installed);
        }
        // Every archive member was length-delimited and SHA-256 checked while
        // unpacking. Persist that fact so normal P&R does not rehash the large
        // architecture database on every process invocation.
        write_verified_marker(
            &temporary,
            &temporary.join("architecture.txdb"),
            &manifest.architecture,
        )?;
        fs::rename(&temporary, &destination)?;
        Ecp5TargetPack::open(destination)
    });
    if result.is_err() && temporary.exists() {
        let _ = fs::remove_dir_all(&temporary);
    }
    result
}

/// Resolves an installed target, downloading its pinned release asset once.
///
/// # Errors
///
/// Returns an error when the device/platform is unpublished, the download or
/// checksum fails, or the pack cannot be installed/opened.
pub fn resolve_ecp5_target(device: &str) -> Result<Ecp5TargetPack, TargetPackError> {
    let platform = host_platform()?;
    let catalog: TargetCatalog = serde_json::from_str(CATALOG)?;
    if catalog.catalog_version != 1 {
        return Err(TargetPackError::new("unsupported embedded target catalog"));
    }
    let entry = catalog
        .targets
        .iter()
        .find(|entry| entry.device == device && entry.platform == platform)
        .ok_or_else(|| {
            TargetPackError::new(format!(
                "no ECP5 target pack is published for {device} on {platform}"
            ))
        })?;
    if entry.bytes == 0 || entry.sha256 == "pending-regeneration" {
        return Err(TargetPackError::new(format!(
            "target catalog entry for {device} has not been published"
        )));
    }
    let cache_root = target_cache_root()?;
    let asset_stem = entry
        .asset
        .strip_suffix(".txpkg.zst")
        .ok_or_else(|| TargetPackError::new("target catalog asset has an invalid suffix"))?;
    let destination = cache_root.join(format!("{}-{asset_stem}", entry.device));
    if destination.is_dir() {
        return Ecp5TargetPack::open(destination);
    }
    fs::create_dir_all(&cache_root)?;
    let download = cache_root.join(format!(".download-{}-{}", std::process::id(), entry.asset));
    let url = format!(
        "https://github.com/fabrica-eda/texo/releases/download/{}/{}",
        entry.release_tag, entry.asset
    );
    eprintln!("downloading ECP5 target pack: {url}");
    let mut response = ureq::get(&url)
        .call()
        .map_err(|error| TargetPackError::new(format!("target download failed: {error}")))?;
    let mut reader = response
        .body_mut()
        .with_config()
        .limit(entry.bytes.saturating_add(1))
        .reader();
    let mut output = File::create(&download)?;
    io::copy(&mut reader, &mut output)?;
    output.sync_all()?;
    if let Err(error) = verify_file(&download, entry.bytes, &entry.sha256) {
        let _ = fs::remove_file(&download);
        return Err(error);
    }
    let installed = install_verified_target_pack(&download, Some(&cache_root));
    let _ = fs::remove_file(&download);
    installed
}

/// Default platform-specific target cache root.
///
/// # Errors
///
/// Returns an error when no supported cache environment variable is present.
pub fn target_cache_root() -> Result<PathBuf, TargetPackError> {
    if let Some(path) = env::var_os("TEXO_TARGET_DIR") {
        return Ok(PathBuf::from(path));
    }
    if let Some(path) = env::var_os("XDG_CACHE_HOME") {
        return Ok(PathBuf::from(path).join("texo/targets"));
    }
    if let Some(path) = env::var_os("HOME") {
        return Ok(PathBuf::from(path).join(".cache/texo/targets"));
    }
    if let Some(path) = env::var_os("LOCALAPPDATA") {
        return Ok(PathBuf::from(path).join("texo/targets"));
    }
    Err(TargetPackError::new(
        "cannot locate target cache; set TEXO_TARGET_DIR",
    ))
}

fn unpack_archive(archive: &Path, destination: &Path) -> Result<(), TargetPackError> {
    let source = BufReader::new(File::open(archive)?);
    let mut decoder = zstd::stream::read::Decoder::new(source)?;
    let mut magic = vec![0; MAGIC.len()];
    decoder.read_exact(&mut magic)?;
    if magic != MAGIC {
        return Err(TargetPackError::new("invalid target-pack magic"));
    }
    let version = read_u32(&mut decoder)?;
    if version != FORMAT_VERSION {
        return Err(TargetPackError::new(format!(
            "unsupported target-pack version {version}"
        )));
    }
    loop {
        let path_length = read_u32(&mut decoder)? as usize;
        if path_length == 0 {
            break;
        }
        if path_length > 4096 {
            return Err(TargetPackError::new("target-pack path is too long"));
        }
        let mut path = vec![0; path_length];
        decoder.read_exact(&mut path)?;
        let relative = PathBuf::from(
            String::from_utf8(path)
                .map_err(|_| TargetPackError::new("target-pack path is not UTF-8"))?,
        );
        validate_relative_path(&relative)?;
        let mode = read_u32(&mut decoder)?;
        let size = read_u64(&mut decoder)?;
        let mut expected_digest = [0_u8; 32];
        decoder.read_exact(&mut expected_digest)?;
        let output_path = destination.join(&relative);
        if let Some(parent) = output_path.parent() {
            fs::create_dir_all(parent)?;
        }
        let mut output = File::create(&output_path)?;
        let mut remaining = size;
        let mut digest = Sha256::new();
        let mut buffer = vec![0_u8; 1024 * 1024];
        while remaining != 0 {
            let chunk = usize::try_from(remaining.min(buffer.len() as u64))
                .expect("chunk is bounded by buffer length");
            decoder.read_exact(&mut buffer[..chunk])?;
            output.write_all(&buffer[..chunk])?;
            digest.update(&buffer[..chunk]);
            remaining -= chunk as u64;
        }
        output.sync_all()?;
        if digest.finalize().as_slice() != expected_digest {
            return Err(TargetPackError::new(format!(
                "target-pack file digest mismatch: {}",
                relative.display()
            )));
        }
        set_mode(&output_path, mode)?;
    }
    Ok(())
}

fn validate_relative_path(path: &Path) -> Result<(), TargetPackError> {
    if path.as_os_str().is_empty()
        || path
            .components()
            .any(|component| !matches!(component, Component::Normal(_)))
    {
        return Err(TargetPackError::new(format!(
            "unsafe target-pack path: {}",
            path.display()
        )));
    }
    Ok(())
}

#[cfg(unix)]
fn set_mode(path: &Path, mode: u32) -> Result<(), TargetPackError> {
    use std::os::unix::fs::PermissionsExt;
    fs::set_permissions(path, fs::Permissions::from_mode(mode & 0o777))?;
    Ok(())
}

#[cfg(not(unix))]
fn set_mode(_path: &Path, _mode: u32) -> Result<(), TargetPackError> {
    Ok(())
}

fn verify_file(path: &Path, bytes: u64, expected: &str) -> Result<(), TargetPackError> {
    if path.metadata()?.len() != bytes {
        return Err(TargetPackError::new(format!(
            "file size mismatch: {}",
            path.display()
        )));
    }
    let actual = sha256_file(path)?;
    if actual != expected {
        return Err(TargetPackError::new(format!(
            "file digest mismatch: {}",
            path.display()
        )));
    }
    Ok(())
}

fn verify_published_archive(path: &Path) -> Result<(), TargetPackError> {
    let platform = host_platform()?;
    let bytes = path.metadata()?.len();
    let catalog: TargetCatalog = serde_json::from_str(CATALOG)?;
    if catalog.catalog_version != 1 {
        return Err(TargetPackError::new("unsupported embedded target catalog"));
    }
    let candidates = catalog
        .targets
        .iter()
        .filter(|entry| entry.platform == platform && entry.bytes == bytes)
        .collect::<Vec<_>>();
    if candidates.is_empty() {
        return Err(TargetPackError::new(format!(
            "target pack is not present in the embedded release catalog: {}",
            path.display()
        )));
    }
    let digest = sha256_file(path)?;
    if !candidates.iter().any(|entry| entry.sha256 == digest) {
        return Err(TargetPackError::new(format!(
            "target pack does not match its published SHA-256: {}",
            path.display()
        )));
    }
    Ok(())
}

fn sha256_file(path: &Path) -> Result<String, TargetPackError> {
    let mut source = BufReader::new(File::open(path)?);
    let mut digest = Sha256::new();
    io::copy(&mut source, &mut digest)?;
    Ok(format!("{:x}", digest.finalize()))
}

fn verify_installed_architecture(
    root: &Path,
    architecture: &Path,
    identity: &FileIdentity,
) -> Result<(), TargetPackError> {
    if architecture.metadata()?.len() != identity.bytes {
        return Err(TargetPackError::new(format!(
            "file size mismatch: {}",
            architecture.display()
        )));
    }
    let expected_marker = verified_marker(identity, architecture)?;
    let marker = fs::read_to_string(root.join(VERIFIED_MARKER)).ok();
    if marker.as_deref() == Some(expected_marker.as_str()) {
        return Ok(());
    }
    verify_file(architecture, identity.bytes, &identity.sha256)?;
    write_verified_marker(root, architecture, identity)
}

fn write_verified_marker(
    root: &Path,
    architecture: &Path,
    identity: &FileIdentity,
) -> Result<(), TargetPackError> {
    fs::write(
        root.join(VERIFIED_MARKER),
        verified_marker(identity, architecture)?,
    )?;
    Ok(())
}

fn verified_marker(
    identity: &FileIdentity,
    architecture: &Path,
) -> Result<String, TargetPackError> {
    let modified = architecture
        .metadata()?
        .modified()?
        .duration_since(UNIX_EPOCH)
        .map_err(|error| TargetPackError::new(error.to_string()))?
        .as_nanos();
    Ok(format!(
        "{} {} {modified}\n",
        identity.sha256, identity.bytes
    ))
}

fn read_u32(reader: &mut impl Read) -> Result<u32, TargetPackError> {
    let mut bytes = [0; 4];
    reader.read_exact(&mut bytes)?;
    Ok(u32::from_le_bytes(bytes))
}

fn read_u64(reader: &mut impl Read) -> Result<u64, TargetPackError> {
    let mut bytes = [0; 8];
    reader.read_exact(&mut bytes)?;
    Ok(u64::from_le_bytes(bytes))
}

fn host_platform() -> Result<&'static str, TargetPackError> {
    match (env::consts::ARCH, env::consts::OS) {
        ("x86_64", "linux") => Ok("x86_64-unknown-linux-gnu"),
        ("aarch64", "linux") => Ok("aarch64-unknown-linux-gnu"),
        (architecture, os) => Err(TargetPackError::new(format!(
            "target packs are not yet published for {architecture}-{os}"
        ))),
    }
}

#[derive(Debug, Deserialize)]
struct TargetCatalog {
    catalog_version: u32,
    targets: Vec<TargetCatalogEntry>,
}

#[derive(Debug, Deserialize)]
struct TargetCatalogEntry {
    device: String,
    platform: String,
    release_tag: String,
    asset: String,
    bytes: u64,
    sha256: String,
}

#[derive(Debug, Deserialize)]
struct PackManifest {
    format_version: u32,
    device: String,
    platform: String,
    artifact_stem: String,
    architecture: FileIdentity,
}

#[derive(Debug, Deserialize)]
struct FileIdentity {
    bytes: u64,
    sha256: String,
}

/// Invalid, unavailable, or unsupported target pack.
#[derive(Debug)]
pub struct TargetPackError {
    message: String,
}

impl TargetPackError {
    fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl fmt::Display for TargetPackError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl Error for TargetPackError {}

impl From<io::Error> for TargetPackError {
    fn from(error: io::Error) -> Self {
        Self::new(error.to_string())
    }
}

impl From<serde_json::Error> for TargetPackError {
    fn from(error: serde_json::Error) -> Self {
        Self::new(error.to_string())
    }
}

#[cfg(test)]
mod tests {
    use std::fs::{self, File};
    use std::io::Write as _;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    use sha2::{Digest, Sha256};

    use super::{
        FORMAT_VERSION, MAGIC, VERIFIED_MARKER, host_platform, install_verified_target_pack,
        validate_relative_path,
    };

    #[test]
    fn archive_paths_cannot_escape_the_install_root() {
        assert!(validate_relative_path(Path::new("database/devices.json")).is_ok());
        assert!(validate_relative_path(Path::new("../escape")).is_err());
        assert!(validate_relative_path(Path::new("/absolute")).is_err());
    }

    #[test]
    fn installs_and_reopens_a_verified_target_pack() {
        let root = temporary_directory("install");
        fs::create_dir(&root).expect("create test directory");
        let architecture = b"tiny architecture fixture";
        let architecture_digest = format!("{:x}", Sha256::digest(architecture));
        let manifest = serde_json::json!({
            "format_version": FORMAT_VERSION,
            "device": "LFE5UM5G-85F",
            "platform": host_platform().expect("supported test host"),
            "artifact_stem": "fixture",
            "architecture": {
                "bytes": architecture.len(),
                "sha256": architecture_digest,
            }
        });
        let files = [
            (
                "manifest.json",
                serde_json::to_vec(&manifest).expect("encode manifest"),
                0o644,
            ),
            ("architecture.txdb", architecture.to_vec(), 0o644),
            ("base.config", b".device LFE5UM5G-85F\n".to_vec(), 0o644),
            ("iodb.json", b"{}\n".to_vec(), 0o644),
            ("database/devices.json", b"{}\n".to_vec(), 0o644),
            ("bin/ecppack", b"#!/bin/sh\n".to_vec(), 0o755),
        ];
        let archive = root.join("fixture.txpkg.zst");
        write_test_archive(&archive, &files);
        let cache = root.join("cache");
        let installed = install_verified_target_pack(&archive, Some(&cache)).expect("install pack");
        assert_eq!(installed.device().expect("read device"), "LFE5UM5G-85F");
        assert!(installed.root.join(VERIFIED_MARKER).is_file());
        assert_eq!(
            install_verified_target_pack(&archive, Some(&cache))
                .expect("reuse pack")
                .root,
            installed.root
        );
        fs::remove_dir_all(root).expect("remove test directory");
    }

    fn write_test_archive(path: &Path, files: &[(&str, Vec<u8>, u32)]) {
        let output = File::create(path).expect("create archive");
        let mut encoder =
            zstd::stream::write::Encoder::new(output, 1).expect("create zstd encoder");
        encoder.write_all(MAGIC).expect("write magic");
        encoder
            .write_all(&FORMAT_VERSION.to_le_bytes())
            .expect("write version");
        for (name, contents, mode) in files {
            encoder
                .write_all(&u32::try_from(name.len()).expect("short name").to_le_bytes())
                .expect("write path length");
            encoder.write_all(name.as_bytes()).expect("write path");
            encoder.write_all(&mode.to_le_bytes()).expect("write mode");
            encoder
                .write_all(
                    &u64::try_from(contents.len())
                        .expect("small fixture")
                        .to_le_bytes(),
                )
                .expect("write size");
            encoder
                .write_all(&Sha256::digest(contents))
                .expect("write digest");
            encoder.write_all(contents).expect("write contents");
        }
        encoder
            .write_all(&0_u32.to_le_bytes())
            .expect("write terminator");
        encoder.finish().expect("finish archive");
    }

    fn temporary_directory(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock")
            .as_nanos();
        std::env::temp_dir().join(format!(
            "texo-target-pack-{label}-{}-{nonce}",
            std::process::id()
        ))
    }
}
