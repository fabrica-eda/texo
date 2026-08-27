//! Direct adapter from Struo's mapped ECP5 object into Texo's problem graph.
//!
//! Struo is revision-pinned by the workspace. Celox is consumed from crates.io;
//! [`celox_frontend_artifact`] keeps the mapped-netlist verification path free
//! of JSON serialization.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

use struo_ir::{ActiveLevel as StruoActiveLevel, ClockEdge as StruoClockEdge};
use struo_target_ecp5::{
    Bit, Control, Ecp5Cell, Ecp5Netlist, MappedPortDirection, PllOutput as StruoPllOutput, Reset,
};
use texo_model::{CellId, CellPinId, Design, ModelError, PinDirection, ResourceKind};

/// Logical signal identity retained from Struo's mapped netlist.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum MappedSignal {
    /// Constant zero network.
    Zero,
    /// Constant one network.
    One,
    /// Numbered mapped wire.
    Wire(u32),
    /// Adapter-local connection introduced while splitting a compound primitive.
    Synthetic(u32),
}

impl From<Bit> for MappedSignal {
    fn from(value: Bit) -> Self {
        match value {
            Bit::Zero => Self::Zero,
            Bit::One => Self::One,
            Bit::Wire(wire) => Self::Wire(wire),
        }
    }
}

/// Target-independent assertion level copied from Struo metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActiveLevel {
    /// Asserted at logic zero.
    Low,
    /// Asserted at logic one.
    High,
}

impl From<StruoActiveLevel> for ActiveLevel {
    fn from(value: StruoActiveLevel) -> Self {
        match value {
            StruoActiveLevel::Low => Self::Low,
            StruoActiveLevel::High => Self::High,
        }
    }
}

/// Target-independent clock edge copied from Struo metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClockEdge {
    /// Low-to-high transition.
    Rising,
    /// High-to-low transition.
    Falling,
}

/// One clock output of an ECP5 `EHXPLLL` primitive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PllOutput {
    /// Primary output (`CLKOP`).
    Clkop,
    /// Secondary output (`CLKOS`).
    Clkos,
    /// Secondary output two (`CLKOS2`).
    Clkos2,
    /// Secondary output three (`CLKOS3`).
    Clkos3,
}

impl PllOutput {
    /// ECP5 primitive port name.
    #[must_use]
    pub const fn port(self) -> &'static str {
        match self {
            Self::Clkop => "CLKOP",
            Self::Clkos => "CLKOS",
            Self::Clkos2 => "CLKOS2",
            Self::Clkos3 => "CLKOS3",
        }
    }
}

impl From<StruoPllOutput> for PllOutput {
    fn from(value: StruoPllOutput) -> Self {
        match value {
            StruoPllOutput::Clkop => Self::Clkop,
            StruoPllOutput::Clkos => Self::Clkos,
            StruoPllOutput::Clkos2 => Self::Clkos2,
            StruoPllOutput::Clkos3 => Self::Clkos3,
        }
    }
}

impl From<StruoClockEdge> for ClockEdge {
    fn from(value: StruoClockEdge) -> Self {
        match value {
            StruoClockEdge::Rising => Self::Rising,
            StruoClockEdge::Falling => Self::Falling,
        }
    }
}

/// Flip-flop reset configuration required during packing and bit generation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResetMetadata {
    /// Assertion level.
    pub active: ActiveLevel,
    /// Whether assertion takes effect asynchronously.
    pub asynchronous: bool,
    /// State loaded on reset.
    pub value: bool,
}

/// Direction of a top-level package port.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PortDirection {
    /// Package input driving the fabric.
    Input,
    /// Fabric output driving the package pin.
    Output,
    /// Bidirectional package pad with separate fabric input/output controls.
    Inout,
}

/// ECP5-specific immutable properties retained beside the generic graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PrimitiveMetadata {
    /// Four-input lookup table configuration.
    Lut4 {
        /// Complete truth table.
        init: u16,
    },
    /// One half of a Struo `CCU2C`, implemented by one `TRELLIS_COMB` BEL.
    CarrySlice {
        /// LUT truth table for this carry slice.
        init: u16,
        /// Whether this slice suppresses its incoming carry.
        inject: bool,
        /// Slice index inside the source `CCU2C`.
        slice: u8,
    },
    /// `TRELLIS_FF` configuration.
    FlipFlop {
        /// Active clock edge.
        edge: ClockEdge,
        /// Optional clock-enable assertion level.
        enable: Option<ActiveLevel>,
        /// Optional local reset configuration.
        reset: Option<ResetMetadata>,
    },
    /// `DP16KD` configuration.
    BlockRam {
        /// Logical number of words.
        depth: u32,
        /// Logical word width.
        word_width: u8,
        /// Configured physical port width.
        physical_width: u8,
        /// Active clock edge.
        edge: ClockEdge,
        /// Write-enable assertion level.
        write_enable: ActiveLevel,
        /// Optional read-enable assertion level.
        read_enable: Option<ActiveLevel>,
    },
    /// Dedicated ECP5 JTAG TAP access block.
    Jtagg {
        /// Whether extension register one is present.
        extension_register_1: bool,
        /// Whether extension register two is present.
        extension_register_2: bool,
    },
    /// User-configured ECP5 phase-locked loop.
    Pll {
        /// PLL output routed into the fabric.
        fabric_output: PllOutput,
        /// PLL output looped back to `CLKFB`.
        feedback_output: PllOutput,
        /// Raw `EHXPLLL` parameters supplied by the user.
        parameters: BTreeMap<String, String>,
        /// Raw `EHXPLLL` attributes supplied by the user.
        attributes: BTreeMap<String, String>,
    },
    /// One bit of a top-level mapped port.
    Port {
        /// Source-level vector port name.
        name: String,
        /// Source-level least-significant-first bit index.
        bit: usize,
        /// Package direction.
        direction: PortDirection,
    },
    /// Dedicated constant source.
    Constant {
        /// Constant value.
        value: bool,
    },
}

/// Source-level vector grouping for imported package-port cells.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ImportedPort {
    /// Source-level port name.
    pub name: String,
    /// Package direction.
    pub direction: PortDirection,
    /// One I/O cell per bit, least-significant first.
    pub bits: Vec<CellId>,
}

/// Generic logical graph plus ECP5 configuration needed by later stages.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ImportedEcp5Design {
    name: String,
    design: Design,
    metadata: BTreeMap<CellId, PrimitiveMetadata>,
    absorbed_inputs: BTreeMap<CellId, BTreeMap<String, bool>>,
    ports: Vec<ImportedPort>,
    carry_pairs: Vec<[CellId; 2]>,
}

impl ImportedEcp5Design {
    /// Mapped top-level design name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Generic logical design consumed by Texo.
    #[must_use]
    pub const fn design(&self) -> &Design {
        &self.design
    }

    /// ECP5 metadata indexed by imported cell ID.
    #[must_use]
    pub const fn metadata(&self) -> &BTreeMap<CellId, PrimitiveMetadata> {
        &self.metadata
    }

    /// Constant input values absorbed into primitive configuration.
    #[must_use]
    pub const fn absorbed_inputs(&self) -> &BTreeMap<CellId, BTreeMap<String, bool>> {
        &self.absorbed_inputs
    }

    /// Source-level vector port grouping.
    #[must_use]
    pub fn ports(&self) -> &[ImportedPort] {
        &self.ports
    }

    /// Two-slice carry primitives that must occupy one ECP5 slice atomically.
    #[must_use]
    pub fn carry_pairs(&self) -> &[[CellId; 2]] {
        &self.carry_pairs
    }

    /// Moves out the generic logical design.
    #[must_use]
    pub fn into_design(self) -> Design {
        self.design
    }
}

/// Imports the exact in-memory Struo mapped object into Texo.
///
/// Routable physical primitive pins are represented explicitly. Constants are
/// folded into LUT INIT or primitive input-mux configuration where legal;
/// residual constant nets receive lazily created constant LUTs. Top-level
/// vector bits become individual I/O cells while their source grouping is
/// retained in [`ImportedPort`].
///
/// # Errors
///
/// Returns an error for inconsistent mapped wiring or an invalid Texo model.
pub fn import_ecp5(netlist: &Ecp5Netlist) -> Result<ImportedEcp5Design, AdapterError> {
    let mut importer = Importer::new();
    for port in netlist.ports() {
        importer.add_port(port)?;
    }
    for cell in netlist.cells() {
        importer.add_primitive(cell)?;
    }
    importer.finish(netlist.name())
}

/// Converts a Struo mapped object into a crates.io Celox artifact in memory.
///
/// # Errors
///
/// Propagates Struo's Celox adapter validation error.
pub fn celox_frontend_artifact(
    netlist: &Ecp5Netlist,
) -> Result<celox::FrontendArtifact, struo_celox::CeloxAdapterError> {
    struo_celox::ecp5_frontend_artifact(netlist)
}

#[derive(Default)]
struct Importer {
    design: Design,
    metadata: BTreeMap<CellId, PrimitiveMetadata>,
    absorbed_inputs: BTreeMap<CellId, BTreeMap<String, bool>>,
    ports: Vec<ImportedPort>,
    pending_inout_ports: BTreeMap<MappedSignal, CellId>,
    carry_pairs: Vec<[CellId; 2]>,
    next_synthetic_signal: u32,
    drivers: BTreeMap<MappedSignal, CellPinId>,
    sinks: BTreeMap<MappedSignal, Vec<CellPinId>>,
}

impl Importer {
    fn new() -> Self {
        Self::default()
    }

    fn add_port(&mut self, port: &struo_target_ecp5::MappedPort) -> Result<(), AdapterError> {
        let direction = match port.direction {
            MappedPortDirection::Input => PortDirection::Input,
            MappedPortDirection::Output => PortDirection::Output,
            MappedPortDirection::Inout => PortDirection::Inout,
        };
        let mut cells = Vec::with_capacity(port.bits.len());
        for (bit_index, mapped_bit) in port.bits.iter().copied().enumerate() {
            let cell = self.add_cell(
                format!("${}[{bit_index}]", port.name),
                ResourceKind::Io,
                PrimitiveMetadata::Port {
                    name: port.name.clone(),
                    bit: bit_index,
                    direction,
                },
            );
            match direction {
                // Project Trellis names PIO ports from the BEL's perspective:
                // `O` carries pad input into fabric, while `I` carries fabric
                // output toward the pad.
                PortDirection::Input => {
                    let pin = self.design.add_pin(cell, "O", PinDirection::Output)?;
                    self.claim_driver(mapped_bit.into(), pin)?;
                }
                PortDirection::Output => {
                    let pin = self.design.add_pin(cell, "I", PinDirection::Input)?;
                    self.add_sink(mapped_bit, pin);
                }
                PortDirection::Inout => {
                    let signal = mapped_bit.into();
                    if self.pending_inout_ports.insert(signal, cell).is_some() {
                        return Err(AdapterError::DuplicateIoPad(signal));
                    }
                }
            }
            cells.push(cell);
        }
        self.ports.push(ImportedPort {
            name: port.name.clone(),
            direction,
            bits: cells,
        });
        Ok(())
    }

    fn add_primitive(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        match primitive {
            Ecp5Cell::Lut4 { .. } => self.add_lut(primitive),
            Ecp5Cell::Ccu2c { .. } => self.add_ccu2c(primitive),
            Ecp5Cell::FlipFlop { .. } => self.add_flip_flop(primitive),
            Ecp5Cell::BlockRam { .. } => self.add_block_ram(primitive),
            Ecp5Cell::TrellisIo { .. } => self.add_trellis_io(primitive),
            Ecp5Cell::Jtagg { .. } => self.add_jtagg(primitive),
            Ecp5Cell::Pll { .. } => self.add_pll(primitive),
        }
    }

    fn add_pll(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::Pll {
            name,
            reference_clock,
            feedback_clock,
            output_clock,
            locked,
            fabric_output,
            feedback_output,
            parameters,
            attributes,
        } = primitive
        else {
            unreachable!("dispatch guarantees EHXPLLL")
        };
        let fabric_output = PllOutput::from(*fabric_output);
        let feedback_output = PllOutput::from(*feedback_output);
        let cell = self.add_cell(
            name,
            ResourceKind::Logic,
            PrimitiveMetadata::Pll {
                fabric_output,
                feedback_output,
                parameters: parameters.clone(),
                attributes: attributes.clone(),
            },
        );
        self.add_input(cell, "CLKI", *reference_clock)?;
        self.add_input(cell, "CLKFB", Bit::Wire(*feedback_clock))?;
        for (name, bit) in [
            ("RST", Bit::Zero),
            ("STDBY", Bit::Zero),
            ("PHASESEL0", Bit::Zero),
            ("PHASESEL1", Bit::Zero),
            ("PHASEDIR", Bit::One),
            ("PHASESTEP", Bit::One),
            ("PHASELOADREG", Bit::One),
            ("PLLWAKESYNC", Bit::Zero),
            ("ENCLKOP", Bit::Zero),
        ] {
            self.add_input(cell, name, bit)?;
        }
        self.add_output(cell, fabric_output.port(), *output_clock)?;
        if fabric_output != feedback_output {
            self.add_output(cell, feedback_output.port(), *feedback_clock)?;
        }
        self.add_output(cell, "LOCK", *locked)
    }

    fn add_jtagg(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::Jtagg {
            name,
            tdo,
            tdi,
            clock,
            run_test_idle,
            shift,
            update,
            reset_n,
            clock_enable,
            extension_register_1,
            extension_register_2,
        } = primitive
        else {
            unreachable!("dispatch guarantees JTAGG")
        };
        for (register, (enabled, bit)) in [*extension_register_1, *extension_register_2]
            .into_iter()
            .zip(*tdo)
            .enumerate()
        {
            if !enabled && constant_value(bit).is_none() {
                return Err(AdapterError::DisabledJtagOutput {
                    register: register + 1,
                    signal: bit.into(),
                });
            }
        }
        let cell = self.add_cell(
            name,
            ResourceKind::Logic,
            PrimitiveMetadata::Jtagg {
                extension_register_1: *extension_register_1,
                extension_register_2: *extension_register_2,
            },
        );
        for (name, bit) in ["JTDO1", "JTDO2"].into_iter().zip(*tdo) {
            self.add_input(cell, name, bit)?;
        }
        for (name, wire) in [
            ("JTDI", *tdi),
            ("JTCK", *clock),
            ("JRTI1", run_test_idle[0]),
            ("JRTI2", run_test_idle[1]),
            ("JSHIFT", *shift),
            ("JUPDATE", *update),
            ("JRSTN", *reset_n),
            ("JCE1", clock_enable[0]),
            ("JCE2", clock_enable[1]),
        ] {
            self.add_output(cell, name, wire)?;
        }
        Ok(())
    }

    fn add_trellis_io(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::TrellisIo {
            pad,
            fabric_output,
            fabric_input,
            tristate,
            ..
        } = primitive
        else {
            unreachable!("dispatch guarantees TRELLIS_IO")
        };
        let pad = MappedSignal::Wire(*pad);
        let cell = self
            .pending_inout_ports
            .remove(&pad)
            .ok_or(AdapterError::UnknownIoPad(pad))?;
        // Keep both controls as routable pins. This mirrors nextpnr's ECP5
        // constant packing: all open-drain pads share the lazily-created GND
        // source for `I`, while a constant `T` can share the VCC/GND source.
        self.add_input(cell, "I", *fabric_output)?;
        self.add_input(cell, "T", *tristate)?;
        self.add_output(cell, "O", *fabric_input)
    }

    fn add_ccu2c(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::Ccu2c {
            name,
            inputs,
            carry_in,
            sums,
            carry_out,
            init,
            inject,
        } = primitive
        else {
            unreachable!("dispatch guarantees CCU2C")
        };
        let internal_carry = MappedSignal::Synthetic(self.next_synthetic_signal);
        self.next_synthetic_signal += 1;
        let mut slices = [CellId(0); 2];
        for slice in 0..2 {
            let (packed_init, absorbed) = pack_carry_inputs(init[slice], inputs[slice]);
            let cell = self.add_cell(
                format!("{name}$slice{slice}"),
                ResourceKind::Lut(4),
                PrimitiveMetadata::CarrySlice {
                    init: packed_init,
                    inject: inject[slice],
                    slice: u8::try_from(slice).expect("CCU2C has two slices"),
                },
            );
            slices[slice] = cell;
            for (index, (pin_name, bit)) in ["A", "B", "C", "D"]
                .into_iter()
                .zip(inputs[slice])
                .enumerate()
            {
                if absorbed[index] {
                    self.record_absorbed_input(
                        cell,
                        pin_name,
                        constant_value(bit).expect("only constants are absorbed"),
                    );
                } else {
                    self.add_input(cell, pin_name, bit)?;
                }
            }
            if slice == 0 {
                self.add_input(cell, "FCI", *carry_in)?;
            } else {
                self.add_signal_input(cell, "FCI", internal_carry)?;
            }
            self.add_output(cell, "F", sums[slice])?;
            if slice == 0 {
                self.add_signal_output(cell, "FCO", internal_carry)?;
            } else {
                self.add_output(cell, "FCO", *carry_out)?;
            }
        }
        self.carry_pairs.push(slices);
        Ok(())
    }

    fn add_lut(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::Lut4 {
            name,
            inputs,
            output,
            init,
        } = primitive
        else {
            unreachable!("dispatch guarantees LUT4")
        };
        let mut packed_init = *init;
        for (index, bit) in inputs.iter().copied().enumerate() {
            if let Some(value) = constant_value(bit) {
                packed_init = fold_lut_input(packed_init, index, value);
            }
        }
        let cell = self.add_cell(
            name,
            ResourceKind::Lut(4),
            PrimitiveMetadata::Lut4 { init: packed_init },
        );
        for (name, bit) in ["A", "B", "C", "D"].into_iter().zip(*inputs) {
            if let Some(value) = constant_value(bit) {
                self.record_absorbed_input(cell, name, value);
            } else {
                self.add_input(cell, name, bit)?;
            }
        }
        // Struo exposes the pre-pack LUT4 port name `Z`. Project Trellis's
        // split-slice BEL uses `F`, matching nextpnr's lut_to_comb packing.
        self.add_output(cell, "F", *output)
    }

    fn add_flip_flop(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::FlipFlop {
            name,
            data,
            output,
            clock,
            edge,
            enable,
            reset,
        } = primitive
        else {
            unreachable!("dispatch guarantees FlipFlop")
        };
        let cell = self.add_cell(
            name,
            ResourceKind::Register,
            PrimitiveMetadata::FlipFlop {
                edge: (*edge).into(),
                enable: enable.map(|control| control.active.into()),
                reset: reset.map(reset_metadata),
            },
        );
        self.add_input(cell, "DI", *data)?;
        self.add_input(cell, "CLK", *clock)?;
        self.add_absorbable_input(
            cell,
            "CE",
            enable.map_or(Bit::One, |control| control.signal),
        )?;
        let lsr = reset.map_or(Bit::Zero, |control| control.signal);
        let lsr_is_inactive = constant_value(lsr).is_some_and(|value| {
            reset.is_none_or(|control| value != matches!(control.active, StruoActiveLevel::High))
        });
        if lsr_is_inactive {
            self.record_absorbed_input(
                cell,
                "LSR",
                constant_value(lsr).expect("inactive check requires a constant"),
            );
        } else {
            self.add_input(cell, "LSR", lsr)?;
        }
        self.add_output(cell, "Q", *output)
    }

    fn add_block_ram(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::BlockRam {
            name,
            depth,
            word_width,
            physical_width,
            write_address,
            write_data,
            write_enable,
            read_address,
            read_data,
            read_enable,
            clock,
            edge,
        } = primitive
        else {
            unreachable!("dispatch guarantees BlockRam")
        };
        let cell = self.add_cell(
            name,
            ResourceKind::Memory,
            PrimitiveMetadata::BlockRam {
                depth: *depth,
                word_width: *word_width,
                physical_width: *physical_width,
                edge: (*edge).into(),
                write_enable: write_enable.active.into(),
                read_enable: read_enable.map(|control| control.active.into()),
            },
        );
        self.add_block_ram_inputs(
            cell,
            write_address,
            write_data,
            *write_enable,
            read_address,
            *read_enable,
            *clock,
        )?;
        for (index, wire) in read_data.iter().copied().enumerate() {
            self.add_output(cell, format!("DOB{index}"), wire)?;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn add_block_ram_inputs(
        &mut self,
        cell: CellId,
        write_address: &[Bit; 14],
        write_data: &[Bit],
        write_enable: Control,
        read_address: &[Bit; 14],
        read_enable: Option<Control>,
        clock: Bit,
    ) -> Result<(), AdapterError> {
        for (index, bit) in write_address.iter().copied().enumerate() {
            self.add_absorbable_input(cell, format!("ADA{index}"), bit)?;
        }
        for index in 0..18 {
            self.add_absorbable_input(
                cell,
                format!("DIA{index}"),
                write_data.get(index).copied().unwrap_or(Bit::Zero),
            )?;
        }
        for (name, bit) in [
            ("CEA", Bit::One),
            ("OCEA", Bit::One),
            ("CLKA", clock),
            ("WEA", write_enable.signal),
            ("RSTA", Bit::Zero),
        ] {
            self.add_absorbable_input(cell, name, bit)?;
        }
        for index in 0..3 {
            self.add_absorbable_input(cell, format!("CSA{index}"), Bit::Zero)?;
        }
        for (index, bit) in read_address.iter().copied().enumerate() {
            self.add_absorbable_input(cell, format!("ADB{index}"), bit)?;
        }
        for index in 0..18 {
            self.add_absorbable_input(cell, format!("DIB{index}"), Bit::Zero)?;
        }
        for (name, bit) in [
            (
                "CEB",
                read_enable.map_or(Bit::One, |control| control.signal),
            ),
            ("OCEB", Bit::One),
            ("CLKB", clock),
            ("WEB", Bit::Zero),
            ("RSTB", Bit::Zero),
        ] {
            self.add_absorbable_input(cell, name, bit)?;
        }
        for index in 0..3 {
            self.add_absorbable_input(cell, format!("CSB{index}"), Bit::Zero)?;
        }
        Ok(())
    }

    fn add_cell(
        &mut self,
        name: impl Into<String>,
        kind: ResourceKind,
        metadata: PrimitiveMetadata,
    ) -> CellId {
        let cell = self.design.add_cell(name, kind);
        self.metadata.insert(cell, metadata);
        cell
    }

    fn add_input(
        &mut self,
        cell: CellId,
        name: impl Into<String>,
        bit: Bit,
    ) -> Result<(), AdapterError> {
        let pin = self.design.add_pin(cell, name, PinDirection::Input)?;
        self.add_sink(bit, pin);
        Ok(())
    }

    fn add_signal_input(
        &mut self,
        cell: CellId,
        name: impl Into<String>,
        signal: MappedSignal,
    ) -> Result<(), AdapterError> {
        let pin = self.design.add_pin(cell, name, PinDirection::Input)?;
        self.sinks.entry(signal).or_default().push(pin);
        Ok(())
    }

    fn add_absorbable_input(
        &mut self,
        cell: CellId,
        name: impl Into<String>,
        bit: Bit,
    ) -> Result<(), AdapterError> {
        let name = name.into();
        if let Some(value) = constant_value(bit) {
            self.record_absorbed_input(cell, name, value);
            Ok(())
        } else {
            self.add_input(cell, name, bit)
        }
    }

    fn record_absorbed_input(&mut self, cell: CellId, name: impl Into<String>, value: bool) {
        self.absorbed_inputs
            .entry(cell)
            .or_default()
            .insert(name.into(), value);
    }

    fn add_constant_driver(&mut self, value: bool) -> Result<CellPinId, AdapterError> {
        let signal = if value {
            MappedSignal::One
        } else {
            MappedSignal::Zero
        };
        if let Some(driver) = self.drivers.get(&signal) {
            return Ok(*driver);
        }
        let cell = self.add_cell(
            if value { "$PACKER_VCC" } else { "$PACKER_GND" },
            ResourceKind::Lut(4),
            PrimitiveMetadata::Constant { value },
        );
        let output = self.design.add_pin(cell, "F", PinDirection::Output)?;
        self.claim_driver(signal, output)?;
        Ok(output)
    }

    fn add_output(
        &mut self,
        cell: CellId,
        name: impl Into<String>,
        wire: u32,
    ) -> Result<(), AdapterError> {
        let pin = self.design.add_pin(cell, name, PinDirection::Output)?;
        self.claim_driver(MappedSignal::Wire(wire), pin)
    }

    fn add_signal_output(
        &mut self,
        cell: CellId,
        name: impl Into<String>,
        signal: MappedSignal,
    ) -> Result<(), AdapterError> {
        let pin = self.design.add_pin(cell, name, PinDirection::Output)?;
        self.claim_driver(signal, pin)
    }

    fn add_sink(&mut self, bit: Bit, pin: CellPinId) {
        self.sinks.entry(bit.into()).or_default().push(pin);
    }

    fn claim_driver(&mut self, signal: MappedSignal, pin: CellPinId) -> Result<(), AdapterError> {
        if self.drivers.insert(signal, pin).is_some() {
            Err(AdapterError::DuplicateDriver(signal))
        } else {
            Ok(())
        }
    }

    fn insert_carry_feedouts(&mut self) -> Result<(), AdapterError> {
        let feedout_signals = self
            .drivers
            .iter()
            .filter_map(|(&signal, &driver)| {
                let needs_feedout = self.sinks.get(&signal).is_none_or(|sinks| {
                    sinks
                        .iter()
                        .any(|sink| self.design.pins()[sink.0].name != "FCI")
                });
                (self.design.pins()[driver.0].name == "FCO" && needs_feedout).then_some(signal)
            })
            .collect::<Vec<_>>();

        for (feedout_index, signal) in feedout_signals.into_iter().enumerate() {
            let original_sinks = self.sinks.remove(&signal).unwrap_or_default();
            let (carry_sinks, general_sinks): (Vec<_>, Vec<_>) = original_sinks
                .into_iter()
                .partition(|sink| self.design.pins()[sink.0].name == "FCI");
            let sum_signal = self.fresh_synthetic_signal();
            let internal_carry = self.fresh_synthetic_signal();
            let continued_carry = self.fresh_synthetic_signal();
            let name = format!("$carry_feedout{feedout_index}");

            let first = self.add_cell(
                format!("{name}$slice0"),
                ResourceKind::Lut(4),
                PrimitiveMetadata::CarrySlice {
                    init: 0,
                    inject: false,
                    slice: 0,
                },
            );
            self.add_signal_input(first, "FCI", signal)?;
            self.add_signal_output(first, "F", sum_signal)?;
            self.add_signal_output(first, "FCO", internal_carry)?;

            let second = self.add_cell(
                format!("{name}$slice1"),
                ResourceKind::Lut(4),
                PrimitiveMetadata::CarrySlice {
                    init: 10,
                    inject: false,
                    slice: 1,
                },
            );
            if !carry_sinks.is_empty() {
                let pin = self.design.add_pin(second, "A", PinDirection::Input)?;
                self.sinks.entry(sum_signal).or_default().push(pin);
            }
            self.add_signal_input(second, "FCI", internal_carry)?;
            let unused_sum = self.fresh_synthetic_signal();
            self.add_signal_output(second, "F", unused_sum)?;
            self.add_signal_output(second, "FCO", continued_carry)?;

            self.sinks.insert(
                signal,
                vec![
                    self.design.cells()[first.0]
                        .pins()
                        .iter()
                        .copied()
                        .find(|pin| self.design.pins()[pin.0].name == "FCI")
                        .expect("feed-out FCI was just inserted"),
                ],
            );
            if !general_sinks.is_empty() {
                self.sinks
                    .entry(sum_signal)
                    .or_default()
                    .extend(general_sinks);
            }
            if !carry_sinks.is_empty() {
                self.sinks.insert(continued_carry, carry_sinks);
            }
            self.carry_pairs.push([first, second]);
        }
        Ok(())
    }

    fn insert_carry_feedins(&mut self) -> Result<(), AdapterError> {
        let mut feedin_sinks = [MappedSignal::Zero, MappedSignal::One]
            .into_iter()
            .flat_map(|signal| {
                self.sinks
                    .get(&signal)
                    .into_iter()
                    .flatten()
                    .copied()
                    .filter(|sink| self.design.pins()[sink.0].name == "FCI")
                    .map(move |sink| (signal, sink))
            })
            .collect::<Vec<_>>();
        feedin_sinks.sort_by_key(|(_, sink)| self.design.pins()[sink.0].cell.0);

        for (feedin_index, (constant, original_sink)) in feedin_sinks.into_iter().enumerate() {
            if let Some(sinks) = self.sinks.get_mut(&constant) {
                sinks.retain(|sink| *sink != original_sink);
            }
            let internal_carry = self.fresh_synthetic_signal();
            let continued_carry = self.fresh_synthetic_signal();
            let name = format!("$carry_feedin{feedin_index}");

            let first = self.add_cell(
                format!("{name}$slice0"),
                ResourceKind::Lut(4),
                PrimitiveMetadata::CarrySlice {
                    init: 0x000a,
                    inject: false,
                    slice: 0,
                },
            );
            self.add_signal_input(first, "A", constant)?;
            self.add_signal_output(first, "FCO", internal_carry)?;

            let second = self.add_cell(
                format!("{name}$slice1"),
                ResourceKind::Lut(4),
                PrimitiveMetadata::CarrySlice {
                    init: 0xffff,
                    inject: true,
                    slice: 1,
                },
            );
            self.add_signal_input(second, "FCI", internal_carry)?;
            self.add_signal_output(second, "FCO", continued_carry)?;
            self.sinks.insert(continued_carry, vec![original_sink]);
            self.carry_pairs.push([first, second]);
        }
        self.sinks.retain(|_, sinks| !sinks.is_empty());
        Ok(())
    }

    fn fresh_synthetic_signal(&mut self) -> MappedSignal {
        let signal = MappedSignal::Synthetic(self.next_synthetic_signal);
        self.next_synthetic_signal += 1;
        signal
    }

    fn finish(mut self, name: &str) -> Result<ImportedEcp5Design, AdapterError> {
        if let Some((&pad, _)) = self.pending_inout_ports.first_key_value() {
            return Err(AdapterError::MissingIoBuffer(pad));
        }
        self.insert_carry_feedins()?;
        self.insert_carry_feedouts()?;
        for (signal, sinks) in std::mem::take(&mut self.sinks) {
            let driver = if let Some(driver) = self.drivers.get(&signal) {
                *driver
            } else {
                match signal {
                    MappedSignal::Zero => self.add_constant_driver(false)?,
                    MappedSignal::One => self.add_constant_driver(true)?,
                    MappedSignal::Wire(_) | MappedSignal::Synthetic(_) => {
                        return Err(AdapterError::MissingDriver(signal));
                    }
                }
            };
            self.design.add_net(signal_name(signal), driver, sinks)?;
        }
        Ok(ImportedEcp5Design {
            name: name.into(),
            design: self.design,
            metadata: self.metadata,
            absorbed_inputs: self.absorbed_inputs,
            ports: self.ports,
            carry_pairs: self.carry_pairs,
        })
    }
}

const fn constant_value(bit: Bit) -> Option<bool> {
    match bit {
        Bit::Zero => Some(false),
        Bit::One => Some(true),
        Bit::Wire(_) => None,
    }
}

fn pack_carry_inputs(mut init: u16, inputs: [Bit; 4]) -> (u16, [bool; 4]) {
    let values = inputs.map(constant_value);
    let mut absorbed = [false; 4];
    for (index, value) in values.into_iter().enumerate() {
        absorbed[index] = match value {
            Some(true) => true,
            Some(false) if index < 2 || values[index ^ 1] == Some(true) => {
                init = fold_lut_input(init, index, false);
                true
            }
            Some(false) | None => false,
        };
    }
    (init, absorbed)
}

fn fold_lut_input(init: u16, input: usize, value: bool) -> u16 {
    let mut folded = 0_u16;
    for output_index in 0..16 {
        let source_index = (output_index & !(1 << input)) | (usize::from(value) << input);
        if init & (1 << source_index) != 0 {
            folded |= 1 << output_index;
        }
    }
    folded
}

fn reset_metadata(reset: Reset) -> ResetMetadata {
    ResetMetadata {
        active: reset.active.into(),
        asynchronous: reset.asynchronous,
        value: reset.value,
    }
}

fn signal_name(signal: MappedSignal) -> String {
    match signal {
        MappedSignal::Zero => "$false".into(),
        MappedSignal::One => "$true".into(),
        MappedSignal::Wire(wire) => format!("$wire{wire}"),
        MappedSignal::Synthetic(signal) => format!("$carry{signal}"),
    }
}

/// Invalid mapped netlist at the Struo-to-Texo boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AdapterError {
    /// Generic logical model construction failed.
    Model(ModelError),
    /// More than one mapped object drove the same signal.
    DuplicateDriver(MappedSignal),
    /// A consumed mapped signal had no driver.
    MissingDriver(MappedSignal),
    /// More than one bidirectional top-level bit names the same physical pad signal.
    DuplicateIoPad(MappedSignal),
    /// A bidirectional top-level bit had no matching `TRELLIS_IO` primitive.
    MissingIoBuffer(MappedSignal),
    /// A `TRELLIS_IO` primitive did not match a bidirectional top-level bit.
    UnknownIoPad(MappedSignal),
    /// A disabled JTAG extension register still had a fabric-driven TDO input.
    DisabledJtagOutput {
        /// One-based extension-register number.
        register: usize,
        /// Fabric signal that would become unobservable.
        signal: MappedSignal,
    },
}

impl fmt::Display for AdapterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Model(error) => write!(f, "invalid imported model: {error}"),
            Self::DuplicateDriver(signal) => {
                write!(f, "mapped signal {signal:?} has multiple drivers")
            }
            Self::MissingDriver(signal) => {
                write!(f, "mapped signal {signal:?} has no driver")
            }
            Self::DuplicateIoPad(signal) => {
                write!(
                    f,
                    "mapped bidirectional pad {signal:?} is declared more than once"
                )
            }
            Self::MissingIoBuffer(signal) => {
                write!(f, "mapped bidirectional pad {signal:?} has no TRELLIS_IO")
            }
            Self::UnknownIoPad(signal) => {
                write!(f, "TRELLIS_IO pad {signal:?} is not a bidirectional port")
            }
            Self::DisabledJtagOutput { register, signal } => write!(
                f,
                "JTAG extension register {register} is disabled but JTDO{register} is driven by {signal:?}"
            ),
        }
    }
}

impl Error for AdapterError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Model(error) => Some(error),
            Self::DuplicateDriver(_)
            | Self::MissingDriver(_)
            | Self::DuplicateIoPad(_)
            | Self::MissingIoBuffer(_)
            | Self::UnknownIoPad(_)
            | Self::DisabledJtagOutput { .. } => None,
        }
    }
}

impl From<ModelError> for AdapterError {
    fn from(value: ModelError) -> Self {
        Self::Model(value)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::num::NonZeroU32;

    use struo_ir::{
        ActiveLevel as StruoActiveLevel, ArithmeticOp, ClockEdge as StruoClockEdge, ComparisonOp,
        EnableControl, MemoryCell, Netlist, RegisterCell, ResetControl,
    };
    use struo_target_ecp5::{
        Bit, JtaggBinding, OpenDrainIo, PllBinding, PllOutput as StruoPllOutput, map_to_ecp5,
        map_to_ecp5_with_jtagg, map_to_ecp5_with_open_drain_ios, map_to_ecp5_with_pll,
    };
    use texo_model::{CellId, ResourceKind};

    use super::{
        ActiveLevel, ClockEdge, PortDirection, PrimitiveMetadata, ResetMetadata,
        celox_frontend_artifact, fold_lut_input, import_ecp5, pack_carry_inputs,
    };

    fn mapped_xor() -> struo_target_ecp5::Ecp5Netlist {
        let mut source = Netlist::new("logic");
        let lhs = source.add_input("lhs");
        let rhs = source.add_input("rhs");
        let value = source.add_xor(lhs, rhs);
        source.add_output("value", value);
        map_to_ecp5(&source).unwrap()
    }

    #[test]
    fn packs_ccu2_constants_with_ecp5_tie_high_rules() {
        assert_eq!(
            pack_carry_inputs(0x96aa, [Bit::Wire(0), Bit::One, Bit::Zero, Bit::One]),
            (0x66aa, [false, true, true, true])
        );
        assert_eq!(
            pack_carry_inputs(0x96aa, [Bit::Wire(0), Bit::Zero, Bit::Zero, Bit::One]),
            (0xaaaa, [false, true, true, true])
        );
        assert_eq!(
            pack_carry_inputs(0x96aa, [Bit::Wire(0), Bit::One, Bit::Zero, Bit::Zero]),
            (0x96aa, [false, true, false, false])
        );
    }

    #[test]
    fn imports_jtagg_pins_and_extension_register_configuration() {
        let mut source = Netlist::new("debug_top");
        for name in [
            "jtag_tdi",
            "jtag_tck",
            "jtag_rti1",
            "jtag_rti2",
            "jtag_shift",
            "jtag_update",
            "jtag_rst_n",
            "jtag_ce1",
            "jtag_ce2",
        ] {
            source.add_input(name);
        }
        let zero = source.add_constant(false);
        source.add_output("jtag_tdo1", zero);
        source.add_output("jtag_tdo2", zero);
        let mut binding = JtaggBinding::with_prefix("jtag");
        binding.extension_register_2 = false;
        let mapped = map_to_ecp5_with_jtagg(&source, &binding).unwrap();

        let imported = import_ecp5(&mapped).unwrap();
        let (cell, metadata) = imported
            .metadata()
            .iter()
            .find(|(_, metadata)| matches!(metadata, PrimitiveMetadata::Jtagg { .. }))
            .unwrap();

        assert!(imported.ports().is_empty());
        assert_eq!(imported.design().cells()[cell.0].kind, ResourceKind::Logic);
        assert_eq!(
            metadata,
            &PrimitiveMetadata::Jtagg {
                extension_register_1: true,
                extension_register_2: false,
            }
        );
        let pins = imported.design().cells()[cell.0]
            .pins()
            .iter()
            .map(|pin| {
                let pin = &imported.design().pins()[pin.0];
                (pin.name.as_str(), pin.direction)
            })
            .collect::<Vec<_>>();
        assert_eq!(
            pins,
            [
                ("JTDO1", texo_model::PinDirection::Input),
                ("JTDO2", texo_model::PinDirection::Input),
                ("JTDI", texo_model::PinDirection::Output),
                ("JTCK", texo_model::PinDirection::Output),
                ("JRTI1", texo_model::PinDirection::Output),
                ("JRTI2", texo_model::PinDirection::Output),
                ("JSHIFT", texo_model::PinDirection::Output),
                ("JUPDATE", texo_model::PinDirection::Output),
                ("JRSTN", texo_model::PinDirection::Output),
                ("JCE1", texo_model::PinDirection::Output),
                ("JCE2", texo_model::PinDirection::Output),
            ]
        );
    }

    #[test]
    fn rejects_a_fabric_driven_tdo_for_a_disabled_jtag_register() {
        let mut source = Netlist::new("debug_top");
        for name in [
            "jtag_tdi",
            "jtag_tck",
            "jtag_rti1",
            "jtag_rti2",
            "jtag_shift",
            "jtag_update",
            "jtag_rst_n",
            "jtag_ce1",
            "jtag_ce2",
        ] {
            source.add_input(name);
        }
        let zero = source.add_constant(false);
        let probe = source.add_input("probe");
        source.add_output("jtag_tdo1", zero);
        source.add_output("jtag_tdo2", probe);
        let mut binding = JtaggBinding::with_prefix("jtag");
        binding.extension_register_2 = false;
        let mapped = map_to_ecp5_with_jtagg(&source, &binding).unwrap();

        let error = import_ecp5(&mapped).unwrap_err();

        assert!(matches!(
            error,
            super::AdapterError::DisabledJtagOutput { register: 2, .. }
        ));
    }

    #[test]
    fn imports_user_configured_pll_pins_and_metadata() {
        let mut source = Netlist::new("pll_top");
        source.add_input("clk");
        source.add_input("clk_250");
        let locked = source.add_input("pll_locked");
        source.add_output("locked", locked);
        let mut binding = PllBinding::new(
            "clk",
            "clk_250",
            "pll_locked",
            StruoPllOutput::Clkos,
            StruoPllOutput::Clkop,
        );
        binding.parameters.insert("CLKI_DIV".into(), "3".into());
        binding
            .attributes
            .insert("FREQUENCY_PIN_CLKOS".into(), "250".into());
        let mapped = map_to_ecp5_with_pll(&source, &binding).unwrap();

        let imported = import_ecp5(&mapped).unwrap();
        let (cell, metadata) = imported
            .metadata()
            .iter()
            .find(|(_, metadata)| matches!(metadata, PrimitiveMetadata::Pll { .. }))
            .unwrap();

        assert_eq!(imported.ports()[0].name, "clk");
        assert_eq!(
            metadata,
            &PrimitiveMetadata::Pll {
                fabric_output: super::PllOutput::Clkos,
                feedback_output: super::PllOutput::Clkop,
                parameters: BTreeMap::from([("CLKI_DIV".into(), "3".into())]),
                attributes: BTreeMap::from([("FREQUENCY_PIN_CLKOS".into(), "250".into())]),
            }
        );
        let pins = imported.design().cells()[cell.0]
            .pins()
            .iter()
            .map(|pin| {
                let pin = &imported.design().pins()[pin.0];
                (pin.name.as_str(), pin.direction)
            })
            .collect::<Vec<_>>();
        assert_eq!(
            pins,
            [
                ("CLKI", texo_model::PinDirection::Input),
                ("CLKFB", texo_model::PinDirection::Input),
                ("RST", texo_model::PinDirection::Input),
                ("STDBY", texo_model::PinDirection::Input),
                ("PHASESEL0", texo_model::PinDirection::Input),
                ("PHASESEL1", texo_model::PinDirection::Input),
                ("PHASEDIR", texo_model::PinDirection::Input),
                ("PHASESTEP", texo_model::PinDirection::Input),
                ("PHASELOADREG", texo_model::PinDirection::Input),
                ("PLLWAKESYNC", texo_model::PinDirection::Input),
                ("ENCLKOP", texo_model::PinDirection::Input),
                ("CLKOS", texo_model::PinDirection::Output),
                ("CLKOP", texo_model::PinDirection::Output),
                ("LOCK", texo_model::PinDirection::Output),
            ]
        );
        assert!(
            imported
                .metadata()
                .values()
                .any(|metadata| matches!(metadata, PrimitiveMetadata::Constant { value: false }))
        );
        assert!(
            imported
                .metadata()
                .values()
                .any(|metadata| matches!(metadata, PrimitiveMetadata::Constant { value: true }))
        );
    }

    #[test]
    fn replicates_the_selected_lut_truth_table_plane() {
        assert_eq!(fold_lut_input(0xaaaa, 0, false), 0x0000);
        assert_eq!(fold_lut_input(0xaaaa, 0, true), 0xffff);
        assert_eq!(fold_lut_input(0xcccc, 1, false), 0x0000);
        assert_eq!(fold_lut_input(0xcccc, 1, true), 0xffff);
    }

    #[test]
    fn folds_lut_constants_without_a_residual_constant_cell() {
        let mapped = mapped_xor();

        let imported = import_ecp5(&mapped).unwrap();

        assert_eq!(imported.name(), "logic");
        assert_eq!(imported.ports().len(), 3);
        let lut = imported
            .design()
            .cells()
            .iter()
            .position(|cell| cell.kind == ResourceKind::Lut(4))
            .map(CellId)
            .unwrap();
        assert!(matches!(
            imported.metadata().get(&lut),
            Some(PrimitiveMetadata::Lut4 { .. })
        ));
        assert_eq!(
            imported.absorbed_inputs()[&lut],
            BTreeMap::from([("C".into(), false), ("D".into(), false)])
        );
        assert!(
            !imported
                .metadata()
                .values()
                .any(|metadata| matches!(metadata, PrimitiveMetadata::Constant { .. }))
        );
        assert!(
            imported
                .design()
                .nets()
                .iter()
                .all(|net| !net.sinks.is_empty())
        );
    }

    #[test]
    fn creates_a_valid_artifact_with_crates_io_celox() {
        let mapped = mapped_xor();

        let artifact = celox_frontend_artifact(&mapped).unwrap();

        assert_eq!(artifact.module_name(), "logic");
        assert_eq!(artifact.port_order().len(), 3);
    }

    #[test]
    fn fuses_an_open_drain_pad_into_one_bidirectional_io_cell() {
        let mut source = Netlist::new("open_drain");
        let sda_i = source.add_input("sda_i");
        let drive_low = source.add_input("drive_low");
        source.add_output("sda_drive_low", drive_low);
        source.add_output("sampled", sda_i);
        let mapped = map_to_ecp5_with_open_drain_ios(
            &source,
            &[OpenDrainIo::new("sda", "sda_i", "sda_drive_low")],
        )
        .unwrap();

        let imported = import_ecp5(&mapped).unwrap();
        let sda = imported
            .ports()
            .iter()
            .find(|port| port.name == "sda")
            .unwrap();
        assert_eq!(sda.direction, PortDirection::Inout);
        assert_eq!(sda.bits.len(), 1);
        let cell = sda.bits[0];
        assert_eq!(imported.design().cells()[cell.0].kind, ResourceKind::Io);
        assert!(matches!(
            imported.metadata().get(&cell),
            Some(PrimitiveMetadata::Port {
                name,
                bit: 0,
                direction: PortDirection::Inout,
            }) if name == "sda"
        ));
        let pins = imported.design().cells()[cell.0]
            .pins()
            .iter()
            .map(|pin| {
                let pin = &imported.design().pins()[pin.0];
                (pin.name.as_str(), pin.direction)
            })
            .collect::<Vec<_>>();
        assert_eq!(
            pins,
            [
                ("I", texo_model::PinDirection::Input),
                ("T", texo_model::PinDirection::Input),
                ("O", texo_model::PinDirection::Output),
            ]
        );
        assert_eq!(
            imported
                .metadata()
                .values()
                .filter(|metadata| matches!(metadata, PrimitiveMetadata::Port { .. }))
                .count(),
            3
        );
    }

    #[test]
    fn splits_ccu2c_into_atomic_carry_slice_pairs() {
        let mut source = Netlist::new("carry");
        let width = NonZeroU32::new(8).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let sum = source
            .add_arithmetic(ArithmeticOp::Add, &lhs, &rhs)
            .unwrap();
        source.add_output_port("sum", &sum).unwrap();

        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();

        assert_eq!(imported.carry_pairs().len(), 6);
        for pair in &imported.carry_pairs()[..4] {
            for (slice, &cell) in pair.iter().enumerate() {
                assert!(matches!(
                    imported.metadata()[&cell],
                    PrimitiveMetadata::CarrySlice {
                        inject: false,
                        slice: actual,
                        ..
                    } if usize::from(actual) == slice
                ));
            }
            let first_pins = imported.design().cells()[pair[0].0]
                .pins()
                .iter()
                .map(|pin| imported.design().pins()[pin.0].name.as_str())
                .collect::<Vec<_>>();
            let second_pins = imported.design().cells()[pair[1].0]
                .pins()
                .iter()
                .map(|pin| imported.design().pins()[pin.0].name.as_str())
                .collect::<Vec<_>>();
            assert!(first_pins.contains(&"FCO"));
            assert!(second_pins.contains(&"FCI"));
        }
        let feedin = &imported.carry_pairs()[4];
        assert_eq!(
            imported.design().cells()[feedin[0].0].name,
            "$carry_feedin0$slice0"
        );
        assert!(matches!(
            imported.metadata()[&feedin[0]],
            PrimitiveMetadata::CarrySlice {
                init: 0x000a,
                inject: false,
                slice: 0,
            }
        ));
        assert!(matches!(
            imported.metadata()[&feedin[1]],
            PrimitiveMetadata::CarrySlice {
                init: 0xffff,
                inject: true,
                slice: 1,
            }
        ));
        assert!(
            imported.design().cells()[feedin[0].0]
                .pins()
                .iter()
                .any(|pin| imported.design().pins()[pin.0].name == "A"),
            "the carry-chain constant must be routed to physical input A"
        );
        let feedout = imported.carry_pairs().last().unwrap();
        assert_eq!(
            imported.design().cells()[feedout[0].0].name,
            "$carry_feedout0$slice0"
        );
        assert!(matches!(
            imported.metadata()[&feedout[0]],
            PrimitiveMetadata::CarrySlice {
                init: 0,
                inject: false,
                slice: 0,
            }
        ));
        assert!(matches!(
            imported.metadata()[&feedout[1]],
            PrimitiveMetadata::CarrySlice {
                init: 10,
                inject: false,
                slice: 1,
            }
        ));
    }

    #[test]
    fn inserts_a_ccu2c_feedout_before_general_carry_consumers() {
        let mut source = Netlist::new("comparison");
        let width = NonZeroU32::new(8).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let result = source
            .add_comparison(ComparisonOp::LessThanUnsigned, &lhs, &rhs)
            .unwrap();
        source.add_output("result", result);

        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();
        let feedout = imported.carry_pairs().last().unwrap();

        assert_eq!(imported.carry_pairs().len(), 6);
        assert!(matches!(
            imported.metadata()[&feedout[0]],
            PrimitiveMetadata::CarrySlice {
                init: 0,
                inject: false,
                slice: 0,
            }
        ));
        assert!(matches!(
            imported.metadata()[&feedout[1]],
            PrimitiveMetadata::CarrySlice {
                init: 10,
                inject: false,
                slice: 1,
            }
        ));
        for net in imported.design().nets() {
            let driver = &imported.design().pins()[net.driver.0];
            if driver.name == "FCO" {
                assert!(
                    net.sinks
                        .iter()
                        .all(|sink| imported.design().pins()[sink.0].name == "FCI")
                );
            }
        }
    }

    #[test]
    fn preserves_flip_flop_controls_and_physical_pin_names() {
        let mut source = Netlist::new("state");
        let data = source.add_input("data");
        let clock = source.add_input("clock");
        let enable = source.add_input("enable");
        let reset = source.add_input("reset_n");
        let output = source.add_register_output("state");
        source.add_register(RegisterCell::new(
            "state",
            output,
            data,
            clock,
            StruoClockEdge::Falling,
            Some(EnableControl {
                signal: enable,
                active: StruoActiveLevel::High,
            }),
            Some(ResetControl {
                signal: reset,
                active: StruoActiveLevel::Low,
                asynchronous: true,
                value: false,
            }),
        ));
        source.add_output("state", output);
        let mapped = map_to_ecp5(&source).unwrap();

        let imported = import_ecp5(&mapped).unwrap();
        let flip_flop = imported
            .design()
            .cells()
            .iter()
            .position(|cell| cell.kind == ResourceKind::Register)
            .map(CellId)
            .unwrap();
        assert_eq!(
            imported.metadata().get(&flip_flop),
            Some(&PrimitiveMetadata::FlipFlop {
                edge: ClockEdge::Falling,
                enable: Some(ActiveLevel::High),
                reset: Some(ResetMetadata {
                    active: ActiveLevel::Low,
                    asynchronous: true,
                    value: false,
                }),
            })
        );
        let pin_names: Vec<_> = imported.design().cells()[flip_flop.0]
            .pins()
            .iter()
            .map(|pin| imported.design().pins()[pin.0].name.as_str())
            .collect();
        assert_eq!(pin_names, ["DI", "CLK", "CE", "LSR", "Q"]);
    }

    #[test]
    fn absorbs_default_flip_flop_controls() {
        let mut source = Netlist::new("state");
        let data = source.add_input("data");
        let clock = source.add_input("clock");
        let output = source.add_register_output("state");
        source.add_register(RegisterCell::new(
            "state",
            output,
            data,
            clock,
            StruoClockEdge::Rising,
            None,
            None,
        ));
        source.add_output("state", output);
        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();
        let flip_flop = imported
            .design()
            .cells()
            .iter()
            .position(|cell| cell.kind == ResourceKind::Register)
            .map(CellId)
            .unwrap();
        let pin_names = imported.design().cells()[flip_flop.0]
            .pins()
            .iter()
            .map(|pin| imported.design().pins()[pin.0].name.as_str())
            .collect::<Vec<_>>();

        assert_eq!(pin_names, ["DI", "CLK", "Q"]);
        assert_eq!(
            imported.absorbed_inputs()[&flip_flop],
            BTreeMap::from([("CE".into(), true), ("LSR".into(), false)])
        );
    }

    #[test]
    fn lazily_creates_a_lut_for_a_residual_constant_net() {
        let mut source = Netlist::new("constant");
        let high = source.add_constant(true);
        source.add_output("high", high);
        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();
        let constant_cells = imported
            .metadata()
            .iter()
            .filter(|(_, metadata)| matches!(metadata, PrimitiveMetadata::Constant { value: true }))
            .collect::<Vec<_>>();

        assert_eq!(constant_cells.len(), 1);
        let cell = *constant_cells[0].0;
        assert_eq!(imported.design().cells()[cell.0].kind, ResourceKind::Lut(4));
        assert_eq!(
            imported.design().pins()[imported.design().cells()[cell.0].pins()[0].0].name,
            "F"
        );
        assert!(
            !imported
                .design()
                .cells()
                .iter()
                .any(|cell| cell.kind == ResourceKind::Constant)
        );
    }

    #[test]
    fn imports_complete_dp16kd_pin_surface() {
        let mut source = Netlist::new("memory");
        let clock = source.add_input("clock");
        let write_enable = source.add_input("write_enable");
        let read_address = (0..2)
            .map(|index| source.add_input(format!("read_address_{index}")))
            .collect();
        let write_address = (0..2)
            .map(|index| source.add_input(format!("write_address_{index}")))
            .collect();
        let write_data = (0..2)
            .map(|index| source.add_input(format!("write_data_{index}")))
            .collect();
        let read_data: Vec<_> = (0..2)
            .map(|index| source.add_memory_output(format!("read_data_{index}")))
            .collect();
        source.add_memory(MemoryCell::new(
            "words",
            4,
            read_address,
            read_data.clone(),
            None,
            write_address,
            write_data,
            EnableControl {
                signal: write_enable,
                active: StruoActiveLevel::High,
            },
            clock,
            StruoClockEdge::Rising,
        ));
        for (index, output) in read_data.into_iter().enumerate() {
            source.add_output(format!("read_data_{index}"), output);
        }
        let mapped = map_to_ecp5(&source).unwrap();

        let imported = import_ecp5(&mapped).unwrap();
        let block_ram = imported
            .design()
            .cells()
            .iter()
            .position(|cell| cell.kind == ResourceKind::Memory)
            .map(CellId)
            .unwrap();

        assert_eq!(imported.design().cells()[block_ram.0].pins().len(), 11);
        assert_eq!(imported.absorbed_inputs()[&block_ram].len(), 71);
        assert!(imported.absorbed_inputs()[&block_ram]["CEA"]);
        assert!(!imported.absorbed_inputs()[&block_ram]["WEB"]);
        assert!(matches!(
            imported.metadata().get(&block_ram),
            Some(PrimitiveMetadata::BlockRam {
                depth: 4,
                word_width: 2,
                physical_width: 2,
                ..
            })
        ));
    }
}
