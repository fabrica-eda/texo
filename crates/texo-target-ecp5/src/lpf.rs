//! ECP5 LPF pin and IO-attribute constraints.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;
use std::io::Read;

use texo_model::CellId;

use crate::PackagePinBinding;

/// Parsed subset of an ECP5 LPF file.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct LpfConstraints {
    locations: BTreeMap<String, String>,
    io_attributes: BTreeMap<String, BTreeMap<String, String>>,
    frequencies_hz: BTreeMap<String, u64>,
    unsupported_commands: Vec<String>,
}

impl LpfConstraints {
    /// Top-level port name to package pin.
    #[must_use]
    pub const fn locations(&self) -> &BTreeMap<String, String> {
        &self.locations
    }

    /// Top-level port name to `IOBUF` key/value attributes.
    #[must_use]
    pub const fn io_attributes(&self) -> &BTreeMap<String, BTreeMap<String, String>> {
        &self.io_attributes
    }

    /// Top-level clock port frequencies, normalized to hertz.
    #[must_use]
    pub const fn frequencies_hz(&self) -> &BTreeMap<String, u64> {
        &self.frequencies_hz
    }

    /// Unsupported commands retained for diagnostics instead of silently lost.
    #[must_use]
    pub fn unsupported_commands(&self) -> &[String] {
        &self.unsupported_commands
    }
}

/// One source-level port and its least-significant-first IO cells.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LogicalPort {
    /// Source-level port name.
    pub name: String,
    /// One IO cell per bit, least-significant first.
    pub bits: Vec<CellId>,
}

/// LPF constraints resolved from port names to logical IO cells.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ResolvedLpf {
    /// Fixed package-pin assignments ready for ECP5 packing.
    pub package_pins: Vec<PackagePinBinding>,
    /// IO attributes indexed by logical IO cell.
    pub io_attributes: BTreeMap<CellId, BTreeMap<String, String>>,
    /// Clock frequencies indexed by the logical IO cell driving the clock.
    pub clock_frequencies_hz: BTreeMap<CellId, u64>,
    /// Unsupported commands copied from the parsed file for diagnostics.
    pub unsupported_commands: Vec<String>,
}

/// Parses nextpnr-compatible `LOCATE COMP`, `IOBUF PORT`, and
/// `FREQUENCY PORT` LPF commands.
///
/// `#` and `//` comments, quoted identifiers, multiple commands per line, and
/// commands spanning lines are supported. Other LPF verbs are retained in
/// [`LpfConstraints::unsupported_commands`].
///
/// # Errors
///
/// Returns an error for input failures, missing semicolons, malformed quoting,
/// malformed supported commands, or duplicate/conflicting constraints.
pub fn parse_lpf(mut reader: impl Read) -> Result<LpfConstraints, LpfError> {
    let mut source = String::new();
    reader
        .read_to_string(&mut source)
        .map_err(|error| LpfError::Io(error.to_string()))?;
    let commands = split_commands(&source)?;
    let mut parsed = LpfConstraints::default();

    for (line, command) in commands {
        let words = tokenize(&command, line)?;
        if words.is_empty() {
            continue;
        }
        match words[0].as_str() {
            "LOCATE" => parse_locate(&mut parsed, &words, line)?,
            "IOBUF" => parse_iobuf(&mut parsed, &words, line)?,
            "FREQUENCY" => parse_frequency(&mut parsed, &words, line)?,
            _ => parsed.unsupported_commands.push(command),
        }
    }
    Ok(parsed)
}

/// Resolves LPF port names into the individual IO cells produced by adapters.
///
/// A single-bit port accepts both `name` and `name[0]`, matching nextpnr's
/// singleton-vector behavior. Multi-bit ports use `name[index]`.
///
/// # Errors
///
/// Returns an error for duplicate logical port names/cells, unknown LPF ports,
/// conflicting aliases, or unconstrained port bits when `allow_unconstrained`
/// is false.
pub fn resolve_lpf_ports(
    constraints: &LpfConstraints,
    ports: &[LogicalPort],
    allow_unconstrained: bool,
) -> Result<ResolvedLpf, LpfError> {
    let (aliases, all_cells) = port_aliases(ports)?;
    let mut package_pins = Vec::new();
    let mut located_cells = BTreeSet::new();
    for (name, pin) in &constraints.locations {
        let cell = aliases
            .get(name)
            .copied()
            .ok_or_else(|| LpfError::UnknownPort(name.clone()))?;
        if !located_cells.insert(cell) {
            return Err(LpfError::DuplicateCellLocation(cell));
        }
        package_pins.push(PackagePinBinding {
            cell,
            pin: pin.clone(),
        });
    }

    if !allow_unconstrained {
        let missing = all_cells
            .difference(&located_cells)
            .copied()
            .collect::<Vec<_>>();
        if !missing.is_empty() {
            return Err(LpfError::UnconstrainedIo(missing));
        }
    }

    let mut io_attributes: BTreeMap<CellId, BTreeMap<String, String>> = BTreeMap::new();
    for (name, attributes) in &constraints.io_attributes {
        let cell = aliases
            .get(name)
            .copied()
            .ok_or_else(|| LpfError::UnknownPort(name.clone()))?;
        let target = io_attributes.entry(cell).or_default();
        for (key, value) in attributes {
            if let Some(previous) = target.insert(key.clone(), value.clone())
                && previous != *value
            {
                return Err(LpfError::ConflictingIoAttribute {
                    cell,
                    key: key.clone(),
                });
            }
        }
    }

    let mut clock_frequencies_hz = BTreeMap::new();
    for (name, &frequency_hz) in &constraints.frequencies_hz {
        let cell = aliases
            .get(name)
            .copied()
            .ok_or_else(|| LpfError::UnknownPort(name.clone()))?;
        if let Some(previous) = clock_frequencies_hz.insert(cell, frequency_hz)
            && previous != frequency_hz
        {
            return Err(LpfError::ConflictingFrequency { cell });
        }
    }

    Ok(ResolvedLpf {
        package_pins,
        io_attributes,
        clock_frequencies_hz,
        unsupported_commands: constraints.unsupported_commands.clone(),
    })
}

/// Resolves borrowed adapter port surfaces without copying adapter-specific
/// types into the target API.
///
/// This accepts iterators such as `imported.ports().iter().map(|port|
/// (port.name.as_str(), port.bits.as_slice()))` from `texo-struo`.
///
/// # Errors
///
/// Returns the same validation errors as [`resolve_lpf_ports`].
pub fn resolve_lpf_port_cells<'a>(
    constraints: &LpfConstraints,
    ports: impl IntoIterator<Item = (&'a str, &'a [CellId])>,
    allow_unconstrained: bool,
) -> Result<ResolvedLpf, LpfError> {
    let ports = ports
        .into_iter()
        .map(|(name, bits)| LogicalPort {
            name: name.into(),
            bits: bits.to_vec(),
        })
        .collect::<Vec<_>>();
    resolve_lpf_ports(constraints, &ports, allow_unconstrained)
}

fn parse_locate(
    parsed: &mut LpfConstraints,
    words: &[String],
    line: usize,
) -> Result<(), LpfError> {
    if words.len() != 5 || words[1] != "COMP" || words[3] != "SITE" {
        return Err(LpfError::MalformedCommand {
            line,
            expected: "LOCATE COMP <port> SITE <pin>",
        });
    }
    if parsed
        .locations
        .insert(words[2].clone(), words[4].clone())
        .is_some()
    {
        return Err(LpfError::DuplicateLocation {
            line,
            port: words[2].clone(),
        });
    }
    Ok(())
}

fn parse_iobuf(parsed: &mut LpfConstraints, words: &[String], line: usize) -> Result<(), LpfError> {
    if words.len() < 4 || words[1] != "PORT" {
        return Err(LpfError::MalformedCommand {
            line,
            expected: "IOBUF PORT <port> <attribute>=<value>...",
        });
    }
    let attributes = parsed.io_attributes.entry(words[2].clone()).or_default();
    for setting in &words[3..] {
        let Some((key, value)) = setting.split_once('=') else {
            return Err(LpfError::MalformedCommand {
                line,
                expected: "IOBUF PORT <port> <attribute>=<value>...",
            });
        };
        if key.is_empty() || value.is_empty() {
            return Err(LpfError::MalformedCommand {
                line,
                expected: "IOBUF PORT <port> <attribute>=<value>...",
            });
        }
        if let Some(previous) = attributes.insert(key.into(), value.into())
            && previous != value
        {
            return Err(LpfError::DuplicateIoAttribute {
                line,
                port: words[2].clone(),
                key: key.into(),
            });
        }
    }
    Ok(())
}

fn parse_frequency(
    parsed: &mut LpfConstraints,
    words: &[String],
    line: usize,
) -> Result<(), LpfError> {
    if words.len() != 5 || words[1] != "PORT" {
        return Err(LpfError::MalformedCommand {
            line,
            expected: "FREQUENCY PORT <port> <value> <HZ|KHZ|MHZ|GHZ>",
        });
    }
    let frequency_hz =
        frequency_hz(&words[3], &words[4]).ok_or_else(|| LpfError::InvalidFrequency {
            line,
            value: words[3].clone(),
            unit: words[4].clone(),
        })?;
    if parsed
        .frequencies_hz
        .insert(words[2].clone(), frequency_hz)
        .is_some()
    {
        return Err(LpfError::DuplicateFrequency {
            line,
            port: words[2].clone(),
        });
    }
    Ok(())
}

fn frequency_hz(value: &str, unit: &str) -> Option<u64> {
    let multiplier = match unit.to_ascii_uppercase().as_str() {
        "HZ" => 1_u128,
        "KHZ" => 1_000,
        "MHZ" => 1_000_000,
        "GHZ" => 1_000_000_000,
        _ => return None,
    };
    let (whole, fraction) = value.split_once('.').unwrap_or((value, ""));
    if whole.is_empty()
        || !whole.bytes().all(|byte| byte.is_ascii_digit())
        || !fraction.bytes().all(|byte| byte.is_ascii_digit())
    {
        return None;
    }
    let denominator = 10_u128.checked_pow(u32::try_from(fraction.len()).ok()?)?;
    let whole = whole.parse::<u128>().ok()?;
    let fraction = if fraction.is_empty() {
        0
    } else {
        fraction.parse::<u128>().ok()?
    };
    let numerator = whole.checked_mul(denominator)?.checked_add(fraction)?;
    let scaled = numerator.checked_mul(multiplier)?;
    if scaled == 0 || scaled % denominator != 0 {
        return None;
    }
    u64::try_from(scaled / denominator).ok()
}

fn port_aliases(
    ports: &[LogicalPort],
) -> Result<(BTreeMap<String, CellId>, BTreeSet<CellId>), LpfError> {
    let mut aliases = BTreeMap::new();
    let mut names = BTreeSet::new();
    let mut cells = BTreeSet::new();
    for port in ports {
        if port.bits.is_empty() {
            return Err(LpfError::EmptyLogicalPort(port.name.clone()));
        }
        if !names.insert(port.name.clone()) {
            return Err(LpfError::DuplicateLogicalPort(port.name.clone()));
        }
        for (index, &cell) in port.bits.iter().enumerate() {
            if !cells.insert(cell) {
                return Err(LpfError::DuplicateLogicalIoCell(cell));
            }
            aliases.insert(format!("{}[{index}]", port.name), cell);
        }
        if port.bits.len() == 1 {
            aliases.insert(port.name.clone(), port.bits[0]);
        }
    }
    Ok((aliases, cells))
}

fn split_commands(source: &str) -> Result<Vec<(usize, String)>, LpfError> {
    let mut commands = Vec::new();
    let mut current = String::new();
    let mut start_line = 1;
    let mut has_content = false;
    let mut quoted = false;
    for (line_index, line) in source.lines().enumerate() {
        let line_number = line_index + 1;
        let line = strip_comment(line);
        for character in line.chars() {
            if character == '"' {
                quoted = !quoted;
            }
            if character == ';' && !quoted {
                if !current.trim().is_empty() {
                    commands.push((start_line, current.trim().into()));
                }
                current.clear();
                has_content = false;
            } else {
                if !has_content && !character.is_whitespace() {
                    start_line = line_number;
                    has_content = true;
                }
                current.push(character);
            }
        }
        current.push(' ');
    }
    if quoted {
        return Err(LpfError::UnterminatedQuote { line: start_line });
    }
    if !current.trim().is_empty() {
        return Err(LpfError::MissingSemicolon { line: start_line });
    }
    Ok(commands)
}

fn strip_comment(line: &str) -> String {
    let mut result = String::new();
    let mut quoted = false;
    let mut characters = line.chars().peekable();
    while let Some(character) = characters.next() {
        if character == '"' {
            quoted = !quoted;
        }
        if !quoted && character == '#' {
            break;
        }
        if !quoted && character == '/' && characters.peek() == Some(&'/') {
            break;
        }
        result.push(character);
    }
    result
}

fn tokenize(command: &str, line: usize) -> Result<Vec<String>, LpfError> {
    let mut words = Vec::new();
    let mut current = String::new();
    let mut active = false;
    let mut quoted = false;
    let mut characters = command.chars();
    while let Some(character) = characters.next() {
        match character {
            '"' => {
                quoted = !quoted;
                active = true;
            }
            '\\' if quoted => {
                let Some(escaped) = characters.next() else {
                    return Err(LpfError::UnterminatedQuote { line });
                };
                current.push(escaped);
                active = true;
            }
            character if character.is_whitespace() && !quoted => {
                if active {
                    words.push(std::mem::take(&mut current));
                    active = false;
                }
            }
            _ => {
                current.push(character);
                active = true;
            }
        }
    }
    if quoted {
        return Err(LpfError::UnterminatedQuote { line });
    }
    if active {
        words.push(current);
    }
    Ok(words)
}

/// LPF syntax or port-resolution error.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LpfError {
    /// Reading the input stream failed.
    Io(String),
    /// A command did not end in a semicolon.
    MissingSemicolon {
        /// First line of the incomplete command.
        line: usize,
    },
    /// A quoted token did not terminate.
    UnterminatedQuote {
        /// First line of the command.
        line: usize,
    },
    /// A supported command had the wrong token surface.
    MalformedCommand {
        /// First line of the command.
        line: usize,
        /// Expected command surface.
        expected: &'static str,
    },
    /// One port had more than one `LOCATE` command.
    DuplicateLocation {
        /// First line of the duplicate command.
        line: usize,
        /// Port name.
        port: String,
    },
    /// One IO attribute was assigned conflicting values.
    DuplicateIoAttribute {
        /// First line of the conflicting command.
        line: usize,
        /// Port name.
        port: String,
        /// Attribute key.
        key: String,
    },
    /// One port had more than one `FREQUENCY` command.
    DuplicateFrequency {
        /// First line of the duplicate command.
        line: usize,
        /// Port name.
        port: String,
    },
    /// A frequency value, unit, or normalized hertz value was invalid.
    InvalidFrequency {
        /// First line of the command.
        line: usize,
        /// Numeric token.
        value: String,
        /// Unit token.
        unit: String,
    },
    /// An LPF command references no known top-level port bit.
    UnknownPort(String),
    /// One logical port has no bits.
    EmptyLogicalPort(String),
    /// Logical port names must be unique.
    DuplicateLogicalPort(String),
    /// One IO cell appeared in multiple logical ports/bits.
    DuplicateLogicalIoCell(CellId),
    /// Both scalar and singleton-vector aliases constrained the same IO cell.
    DuplicateCellLocation(CellId),
    /// Strict resolution found unconstrained logical IO cells.
    UnconstrainedIo(Vec<CellId>),
    /// Aliases assigned conflicting IO attributes to one cell.
    ConflictingIoAttribute {
        /// Logical IO cell.
        cell: CellId,
        /// Attribute key.
        key: String,
    },
    /// Scalar and singleton aliases assigned different frequencies.
    ConflictingFrequency {
        /// Logical IO cell.
        cell: CellId,
    },
}

impl fmt::Display for LpfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "failed to read LPF: {error}"),
            Self::MissingSemicolon { line } => {
                write!(f, "LPF command starting on line {line} has no semicolon")
            }
            Self::UnterminatedQuote { line } => {
                write!(
                    f,
                    "LPF command starting on line {line} has an unterminated quote"
                )
            }
            Self::MalformedCommand { line, expected } => {
                write!(
                    f,
                    "malformed LPF command on line {line}; expected `{expected}`"
                )
            }
            Self::DuplicateLocation { line, port } => {
                write!(
                    f,
                    "LPF port `{port}` has a duplicate location on line {line}"
                )
            }
            Self::DuplicateIoAttribute { line, port, key } => write!(
                f,
                "LPF port `{port}` has a conflicting `{key}` attribute on line {line}"
            ),
            Self::DuplicateFrequency { line, port } => write!(
                f,
                "LPF port `{port}` has a duplicate frequency on line {line}"
            ),
            Self::InvalidFrequency { line, value, unit } => {
                write!(f, "invalid LPF frequency `{value} {unit}` on line {line}")
            }
            Self::UnknownPort(port) => write!(f, "LPF references unknown port `{port}`"),
            Self::EmptyLogicalPort(port) => write!(f, "logical port `{port}` has no bits"),
            Self::DuplicateLogicalPort(port) => write!(f, "duplicate logical port `{port}`"),
            Self::DuplicateLogicalIoCell(cell) => {
                write!(
                    f,
                    "logical IO cell {} occurs in more than one port bit",
                    cell.0
                )
            }
            Self::DuplicateCellLocation(cell) => {
                write!(
                    f,
                    "logical IO cell {} has more than one LPF location",
                    cell.0
                )
            }
            Self::UnconstrainedIo(cells) => {
                write!(f, "{} logical IO cell(s) are unconstrained", cells.len())
            }
            Self::ConflictingIoAttribute { cell, key } => write!(
                f,
                "logical IO cell {} has conflicting `{key}` attributes",
                cell.0
            ),
            Self::ConflictingFrequency { cell } => write!(
                f,
                "logical IO cell {} has conflicting clock frequencies",
                cell.0
            ),
        }
    }
}

impl Error for LpfError {}

#[cfg(test)]
mod tests {
    use texo_model::CellId;

    use super::{LogicalPort, LpfError, parse_lpf, resolve_lpf_ports};

    #[test]
    fn parses_comments_multiline_locations_and_iobuf_attributes() {
        let parsed = parse_lpf(
            br#"
                # board pins
                LOCATE COMP "led[0]"
                    SITE "A10"; // first LED
                IOBUF PORT "led[0]" IO_TYPE=LVCMOS33 DRIVE=8;
                FREQUENCY PORT "clk" 25 MHZ;
            "#
            .as_slice(),
        )
        .unwrap();

        assert_eq!(parsed.locations()["led[0]"], "A10");
        assert_eq!(parsed.io_attributes()["led[0]"]["IO_TYPE"], "LVCMOS33");
        assert_eq!(parsed.io_attributes()["led[0]"]["DRIVE"], "8");
        assert_eq!(parsed.frequencies_hz()["clk"], 25_000_000);
        assert!(parsed.unsupported_commands().is_empty());
    }

    #[test]
    fn resolves_vector_and_singleton_aliases() {
        let parsed = parse_lpf(
            br#"
                LOCATE COMP "led[0]" SITE "A10";
                LOCATE COMP "led[1]" SITE "B10";
                LOCATE COMP "clk" SITE "P3";
                IOBUF PORT "clk" IO_TYPE=LVCMOS33;
            "#
            .as_slice(),
        )
        .unwrap();
        let resolved = resolve_lpf_ports(
            &parsed,
            &[
                LogicalPort {
                    name: "led".into(),
                    bits: vec![CellId(4), CellId(5)],
                },
                LogicalPort {
                    name: "clk".into(),
                    bits: vec![CellId(6)],
                },
            ],
            false,
        )
        .unwrap();

        assert_eq!(resolved.package_pins.len(), 3);
        assert_eq!(resolved.io_attributes[&CellId(6)]["IO_TYPE"], "LVCMOS33");
    }

    #[test]
    fn strict_resolution_reports_unconstrained_bits() {
        let parsed = parse_lpf(b"LOCATE COMP led[0] SITE A10;".as_slice()).unwrap();
        let error = resolve_lpf_ports(
            &parsed,
            &[LogicalPort {
                name: "led".into(),
                bits: vec![CellId(0), CellId(1)],
            }],
            false,
        )
        .unwrap_err();

        assert_eq!(error, LpfError::UnconstrainedIo(vec![CellId(1)]));
    }

    #[test]
    fn rejects_conflicting_duplicate_locations() {
        let error = parse_lpf(b"LOCATE COMP clk SITE A10; LOCATE COMP clk SITE B10;".as_slice())
            .unwrap_err();

        assert!(matches!(error, LpfError::DuplicateLocation { .. }));
    }

    #[test]
    fn reports_unterminated_quotes_at_the_command_line() {
        let error = parse_lpf(b"\nLOCATE COMP \"clk SITE A10;".as_slice()).unwrap_err();

        assert_eq!(error, LpfError::UnterminatedQuote { line: 2 });
    }

    #[test]
    fn parses_exact_decimal_frequencies_and_rejects_fractional_hertz() {
        let parsed = parse_lpf(b"FREQUENCY PORT clk 12.5 MHZ;".as_slice()).unwrap();
        assert_eq!(parsed.frequencies_hz()["clk"], 12_500_000);

        assert!(matches!(
            parse_lpf(b"FREQUENCY PORT clk 0.1 HZ;".as_slice()),
            Err(LpfError::InvalidFrequency { .. })
        ));
    }
}
