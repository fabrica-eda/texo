# Binary checkpoints

`texo pnr` writes `.txcp` checkpoints by default. The file consists of the
8-byte `TEXOCP\x01\n` header followed by one checksummed Zstd frame containing
CBOR. The logical schema remains version 3: routes, placement, measured timing,
coverage exceptions and verification evidence are preserved.

The writer serializes directly into the compressor, without constructing or
writing an expanded JSON document. It writes a same-directory temporary file,
synchronizes the finished file, atomically replaces the destination, then
synchronizes the containing directory on Unix. A serialization failure leaves
an existing checkpoint intact. Readers consume the frame footer so truncated
or corrupt compressed data cannot masquerade as a complete checkpoint.

Resume, bitgen and visualization recognize the header and continue to accept
legacy JSON input. New checkpoint destinations with a `.json` suffix are
rejected. Small configuration and summary files remain JSON.

Convert an existing checkpoint without changing its saved evidence:

```sh
texo checkpoint-convert old.json design.txcp
```

Conversion rereads the result and verifies full content equality. It does not
requalify timing or replace the need for fresh STA when resuming placement and
routing. Initial placement, LUT/FF pairs and route hints can also use the binary
container, avoiding expanded route-hint JSON files.
