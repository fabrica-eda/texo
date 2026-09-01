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
    Bit, Control, Ecp5Cell, Ecp5MemoryImplementation, Ecp5Netlist, MappedPortDirection,
    PllOutput as StruoPllOutput, Reset,
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
    /// Dedicated internal feedback output (`CLKINTFB`).
    Clkintfb,
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
            Self::Clkintfb => "CLKINTFB",
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
            StruoPllOutput::Clkintfb => Self::Clkintfb,
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

/// Independently clocked second-port configuration of a true-dual-port RAM.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BlockRamPortMetadata {
    /// Active clock edge.
    pub edge: ClockEdge,
    /// Write-enable assertion level.
    pub write_enable: ActiveLevel,
    /// Optional read-enable assertion level.
    pub read_enable: Option<ActiveLevel>,
}

/// Physical role of one cell in a packed `TRELLIS_DPR16X4` macro.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DistributedRamRole {
    /// One of the four asynchronous-read LUT-RAM bits.
    Data(u8),
    /// The `TRELLIS_RAMW` write-address/data distributor.
    WritePort,
    /// One of the two LUT slots reserved by the RAM write machinery.
    WriteBlocker,
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
        /// Independently clocked second port, when this is true dual port.
        second_port: Option<BlockRamPortMetadata>,
    },
    /// One physical cell inside a `TRELLIS_DPR16X4` distributed-RAM macro.
    DistributedRam {
        /// Cell role within the seven-cell physical macro.
        role: DistributedRamRole,
        /// Active write-clock edge.
        edge: ClockEdge,
        /// Write-enable assertion level.
        write_enable: ActiveLevel,
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
    wide_lut_clusters: Vec<Vec<CellId>>,
    distributed_ram_clusters: Vec<DistributedRamCluster>,
}

/// Seven cells that implement one ECP5 `TRELLIS_DPR16X4` primitive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DistributedRamCluster {
    /// Four LUT-RAM data cells in physical K0..K3 order.
    pub data: [CellId; 4],
    /// Two otherwise-unused LUT slots occupied by RAMW.
    pub blockers: [CellId; 2],
    /// Dedicated write-address/data distribution BEL.
    pub write_port: CellId,
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

    /// LUT4 cells constrained into one dedicated LUT5, LUT6, or LUT7 cluster.
    ///
    /// Two-cell clusters occupy one ECP5 slice. Four-cell clusters occupy two
    /// adjacent slices. Eight-cell clusters occupy one complete PLC. All
    /// cluster sizes preserve the physical order required by the
    /// `PFUMX`/`L6MUX21` cascade.
    #[must_use]
    pub fn wide_lut_clusters(&self) -> &[Vec<CellId>] {
        &self.wide_lut_clusters
    }

    /// Distributed-RAM macros that must occupy one PLC atomically.
    #[must_use]
    pub fn distributed_ram_clusters(&self) -> &[DistributedRamCluster] {
        &self.distributed_ram_clusters
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
    for cell in netlist
        .cells()
        .iter()
        .filter(|cell| !matches!(cell, Ecp5Cell::PfuMux { .. } | Ecp5Cell::L6Mux21 { .. }))
    {
        importer.add_primitive(cell)?;
    }
    for cell in netlist
        .cells()
        .iter()
        .filter(|cell| matches!(cell, Ecp5Cell::PfuMux { .. }))
    {
        importer.add_pfu_mux(cell)?;
    }
    for cell in netlist
        .cells()
        .iter()
        .filter(|cell| matches!(cell, Ecp5Cell::L6Mux21 { .. }))
    {
        importer.add_l6_mux(cell)?;
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
    pending_wide_muxes: BTreeMap<u32, Vec<CellId>>,
    wide_lut_clusters: Vec<Vec<CellId>>,
    distributed_ram_clusters: Vec<DistributedRamCluster>,
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
            Ecp5Cell::PfuMux { .. } | Ecp5Cell::L6Mux21 { .. } => {
                unreachable!("wide muxes are packed after their LUT4 drivers")
            }
            Ecp5Cell::Ccu2c { .. } => self.add_ccu2c(primitive),
            Ecp5Cell::FlipFlop { .. } => self.add_flip_flop(primitive),
            Ecp5Cell::BlockRam {
                implementation: Ecp5MemoryImplementation::Block,
                ..
            } => self.add_block_ram(primitive),
            Ecp5Cell::BlockRam {
                implementation: Ecp5MemoryImplementation::Distributed,
                ..
            } => self.add_distributed_ram(primitive),
            Ecp5Cell::TrellisIo { .. } => self.add_trellis_io(primitive),
            Ecp5Cell::Jtagg { .. } => self.add_jtagg(primitive),
            Ecp5Cell::Pll { .. } => self.add_pll(primitive),
        }
    }

    fn add_pfu_mux(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::PfuMux {
            name,
            lut_true,
            lut_false,
            select,
            output,
        } = primitive
        else {
            unreachable!("dispatch guarantees PFUMX")
        };
        let lut_false_wire = wide_mux_wire(name, *lut_false, "BLUT")?;
        let root = self.wide_lut_driver(name, *lut_false, "BLUT", "F")?;
        // F and OFX alias one fabric-facing wire. If the BLUT result also has
        // an ordinary consumer, preserve that F cell and replicate only its
        // truth table/input plane for the dedicated PFUMX root.
        let root = if self
            .sinks
            .get(&MappedSignal::Wire(lut_false_wire))
            .is_some_and(|sinks| !sinks.is_empty())
        {
            self.clone_wide_lut_root(name, root)?
        } else {
            root
        };
        let child = self.wide_lut_driver(name, *lut_true, "ALUT", "F")?;
        if root == child {
            return Err(AdapterError::InvalidWideLut {
                cell: name.clone(),
                reason: "ALUT and BLUT are driven by the same LUT4".into(),
            });
        }
        self.add_input(root, "F1", *lut_true)?;
        self.add_input(root, "M", *select)?;
        self.add_output(root, "OFX", *output)?;
        if self
            .pending_wide_muxes
            .insert(*output, vec![root, child])
            .is_some()
        {
            return Err(AdapterError::InvalidWideLut {
                cell: name.clone(),
                reason: format!("PFUMX output wire {output} is repeated"),
            });
        }
        Ok(())
    }

    fn clone_wide_lut_root(
        &mut self,
        wide_cell: &str,
        source: CellId,
    ) -> Result<CellId, AdapterError> {
        let source_cell = &self.design.cells()[source.0];
        let metadata =
            self.metadata
                .get(&source)
                .cloned()
                .ok_or_else(|| AdapterError::InvalidWideLut {
                    cell: wide_cell.into(),
                    reason: "BLUT driver has no LUT4 metadata".into(),
                })?;
        if !matches!(metadata, PrimitiveMetadata::Lut4 { .. }) {
            return Err(AdapterError::InvalidWideLut {
                cell: wide_cell.into(),
                reason: "BLUT driver metadata is not LUT4".into(),
            });
        }
        let source_name = source_cell.name.clone();
        let inputs = source_cell
            .pins()
            .iter()
            .copied()
            .filter(|pin| self.design.pins()[pin.0].direction == PinDirection::Input)
            .map(|pin| {
                let source_pin = &self.design.pins()[pin.0];
                let signal = self
                    .sinks
                    .iter()
                    .find_map(|(&signal, sinks)| sinks.contains(&pin).then_some(signal))
                    .ok_or_else(|| AdapterError::InvalidWideLut {
                        cell: wide_cell.into(),
                        reason: format!("BLUT input {} has no mapped signal", source_pin.name),
                    })?;
                Ok((source_pin.name.clone(), signal))
            })
            .collect::<Result<Vec<_>, AdapterError>>()?;
        let absorbed = self.absorbed_inputs.get(&source).cloned();
        let clone = self.add_cell(
            format!("{source_name}$wide_mux_clone"),
            ResourceKind::Lut(4),
            metadata,
        );
        for (name, signal) in inputs {
            self.add_signal_input(clone, name, signal)?;
        }
        if let Some(absorbed) = absorbed {
            self.absorbed_inputs.insert(clone, absorbed);
        }
        Ok(clone)
    }

    fn add_l6_mux(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::L6Mux21 {
            name,
            data_zero,
            data_one,
            select,
            output,
        } = primitive
        else {
            unreachable!("dispatch guarantees L6MUX21")
        };
        let zero_wire = wide_mux_wire(name, *data_zero, "D0")?;
        let one_wire = wide_mux_wire(name, *data_one, "D1")?;
        let zero = self.pending_wide_muxes.remove(&zero_wire).ok_or_else(|| {
            AdapterError::InvalidWideLut {
                cell: name.clone(),
                reason: format!("D0 wire {zero_wire} is not driven by a packed wide mux"),
            }
        })?;
        let one = self.pending_wide_muxes.remove(&one_wire).ok_or_else(|| {
            AdapterError::InvalidWideLut {
                cell: name.clone(),
                reason: format!("D1 wire {one_wire} is not driven by a packed wide mux"),
            }
        })?;
        if zero.len() != one.len() || !matches!(zero.len(), 2 | 4) {
            return Err(AdapterError::InvalidWideLut {
                cell: name.clone(),
                reason: format!(
                    "D0 and D1 have incompatible {}- and {}-LUT clusters",
                    zero.len(),
                    one.len()
                ),
            });
        }
        if zero.iter().any(|cell| one.contains(cell)) {
            return Err(AdapterError::InvalidWideLut {
                cell: name.clone(),
                reason: "D0 and D1 wide-mux clusters overlap".into(),
            });
        }

        // This is the same packed representation used by nextpnr-ecp5. The
        // last LUT of the D1 half owns this L6MUX21's ports. For LUT6 this is
        // the upper LUT of its D1 PFUMX; for LUT7 it is the otherwise unused
        // upper LUT of its D1 LUT6's D0 PFUMX. Physical order is always D1
        // followed by D0.
        let root = *one.last().expect("wide-mux halves are nonempty");
        self.add_input(root, "FXA", *data_zero)?;
        self.add_input(root, "FXB", *data_one)?;
        self.add_input(root, "M", *select)?;
        self.add_output(root, "OFX", *output)?;
        let mut cluster = one;
        cluster.extend(zero);
        if self.pending_wide_muxes.insert(*output, cluster).is_some() {
            return Err(AdapterError::InvalidWideLut {
                cell: name.clone(),
                reason: format!("L6MUX21 output wire {output} is repeated"),
            });
        }
        Ok(())
    }

    fn wide_lut_driver(
        &self,
        wide_cell: &str,
        bit: Bit,
        input: &str,
        output: &str,
    ) -> Result<CellId, AdapterError> {
        let wire = wide_mux_wire(wide_cell, bit, input)?;
        let signal = MappedSignal::Wire(wire);
        let driver = self
            .drivers
            .get(&signal)
            .ok_or_else(|| AdapterError::InvalidWideLut {
                cell: wide_cell.into(),
                reason: format!("{input} wire {wire} has no LUT4 driver"),
            })?;
        let pin = &self.design.pins()[driver.0];
        if pin.name != output
            || self.design.cells()[pin.cell.0].kind != ResourceKind::Lut(4)
            || !matches!(
                self.metadata.get(&pin.cell),
                Some(PrimitiveMetadata::Lut4 { .. })
            )
        {
            return Err(AdapterError::InvalidWideLut {
                cell: wide_cell.into(),
                reason: format!("{input} wire {wire} is not driven by a LUT4 {output} pin"),
            });
        }
        Ok(pin.cell)
    }

    fn add_pll(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::Pll {
            name,
            reference_clock,
            feedback_clock,
            output_clock,
            additional_output_clocks,
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
        for (output, wire) in additional_output_clocks {
            self.add_output(cell, PllOutput::from(*output).port(), *wire)?;
        }
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
            implementation: Ecp5MemoryImplementation::Block,
            depth,
            word_width,
            physical_width,
            write_address,
            write_data,
            write_enable,
            read_address,
            read_data,
            read_enable,
            clock_enable,
            clock,
            edge,
            second_port,
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
                second_port: second_port.as_ref().map(|port| BlockRamPortMetadata {
                    edge: port.edge.into(),
                    write_enable: port.write_enable.active.into(),
                    read_enable: port.read_enable.map(|control| control.active.into()),
                }),
            },
        );
        self.add_block_ram_inputs(
            cell,
            write_address,
            write_data,
            *write_enable,
            read_address,
            *read_enable,
            *clock_enable,
            *clock,
            second_port.as_ref().map(|port| port.address.as_ref()),
            second_port.as_ref().map(|port| port.write_data.as_slice()),
            second_port.as_ref().map(|port| port.write_enable),
            second_port.as_ref().and_then(|port| port.clock_enable),
            second_port.as_ref().map(|port| port.clock),
        )?;
        let primary_output = if second_port.is_some() { "DOA" } else { "DOB" };
        for (index, wire) in read_data.iter().copied().enumerate() {
            self.add_output(cell, format!("{primary_output}{index}"), wire)?;
        }
        if let Some(port) = second_port {
            for (index, wire) in port.read_data.iter().copied().enumerate() {
                self.add_output(cell, format!("DOB{index}"), wire)?;
            }
        }
        Ok(())
    }

    fn add_distributed_ram(&mut self, primitive: &Ecp5Cell) -> Result<(), AdapterError> {
        let Ecp5Cell::BlockRam {
            name,
            implementation: Ecp5MemoryImplementation::Distributed,
            write_address,
            write_data,
            write_enable,
            read_address,
            read_data,
            clock,
            edge,
            ..
        } = primitive
        else {
            unreachable!("dispatch guarantees distributed RAM")
        };
        debug_assert_eq!(write_data.len(), 4);
        debug_assert_eq!(read_data.len(), 4);

        let metadata = |role| PrimitiveMetadata::DistributedRam {
            role,
            edge: (*edge).into(),
            write_enable: write_enable.active.into(),
        };
        let write_port = self.add_cell(
            format!("{name}$RAMW_SLICE"),
            ResourceKind::Logic,
            metadata(DistributedRamRole::WritePort),
        );
        for (pin, bit) in [
            ("D0", write_address[0]),
            ("B0", write_address[1]),
            ("C0", write_address[2]),
            ("A0", write_address[3]),
            ("C1", write_data[0]),
            ("A1", write_data[1]),
            ("D1", write_data[2]),
            ("B1", write_data[3]),
        ] {
            self.add_input(write_port, pin, bit)?;
        }

        let address_signals: [MappedSignal; 4] =
            std::array::from_fn(|_| self.fresh_synthetic_signal());
        let data_signals: [MappedSignal; 4] =
            std::array::from_fn(|_| self.fresh_synthetic_signal());
        for (index, signal) in address_signals.into_iter().enumerate() {
            self.add_signal_output(write_port, format!("WADO{index}"), signal)?;
        }
        for (index, signal) in data_signals.into_iter().enumerate() {
            self.add_signal_output(write_port, format!("WDO{index}"), signal)?;
        }

        let mut data = [CellId(0); 4];
        for index in 0..4 {
            let cell = self.add_cell(
                format!("{name}$DPRAM_COMB{index}"),
                ResourceKind::Lut(4),
                metadata(DistributedRamRole::Data(
                    u8::try_from(index).expect("distributed RAM has four data bits"),
                )),
            );
            data[index] = cell;
            for (pin, bit) in [
                ("D", read_address[0]),
                ("B", read_address[1]),
                ("C", read_address[2]),
                ("A", read_address[3]),
                ("WRE", write_enable.signal),
                ("WCK", *clock),
            ] {
                self.add_input(cell, pin, bit)?;
            }
            for (address, signal) in address_signals.into_iter().enumerate() {
                self.add_signal_input(cell, format!("WAD{address}"), signal)?;
            }
            self.add_signal_input(cell, "WD", data_signals[index])?;
            self.add_output(cell, "F", read_data[index])?;
        }

        let blockers = std::array::from_fn(|index| {
            self.add_cell(
                format!("{name}$RAMW_BLOCK{index}"),
                ResourceKind::Lut(4),
                metadata(DistributedRamRole::WriteBlocker),
            )
        });
        self.distributed_ram_clusters.push(DistributedRamCluster {
            data,
            blockers,
            write_port,
        });
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
        clock_enable: Option<Control>,
        clock: Bit,
        second_address: Option<&[Bit; 14]>,
        second_write_data: Option<&[Bit]>,
        second_write_enable: Option<Control>,
        second_clock_enable: Option<Control>,
        second_clock: Option<Bit>,
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
            (
                "CEA",
                clock_enable.map_or(Bit::One, |control| control.signal),
            ),
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
        let port_b_address = second_address.unwrap_or(read_address);
        for (index, bit) in port_b_address.iter().copied().enumerate() {
            self.add_absorbable_input(cell, format!("ADB{index}"), bit)?;
        }
        for index in 0..18 {
            self.add_absorbable_input(
                cell,
                format!("DIB{index}"),
                second_write_data
                    .and_then(|data| data.get(index).copied())
                    .unwrap_or(Bit::Zero),
            )?;
        }
        for (name, bit) in [
            (
                "CEB",
                second_address.map_or_else(
                    || read_enable.map_or(Bit::One, |control| control.signal),
                    |_| second_clock_enable.map_or(Bit::One, |control| control.signal),
                ),
            ),
            ("OCEB", Bit::One),
            ("CLKB", second_clock.unwrap_or(clock)),
            (
                "WEB",
                second_write_enable.map_or(Bit::Zero, |enable| enable.signal),
            ),
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
        let mut feedin_sinks = self
            .sinks
            .keys()
            .copied()
            .filter(|signal| {
                self.drivers
                    .get(signal)
                    .is_none_or(|driver| self.design.pins()[driver.0].name != "FCO")
            })
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
        self.wide_lut_clusters
            .extend(std::mem::take(&mut self.pending_wide_muxes).into_values());
        self.wide_lut_clusters
            .sort_unstable_by_key(|cluster| cluster.iter().copied().min());
        for (signal, mut sinks) in std::mem::take(&mut self.sinks) {
            // Sink order has no logical meaning. Keep it independent of the
            // primitive traversal that discovered the pins so analytical
            // adjacency and floating-point accumulation are canonical too.
            sinks.sort_unstable();
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
            wide_lut_clusters: self.wide_lut_clusters,
            distributed_ram_clusters: self.distributed_ram_clusters,
        })
    }
}

fn wide_mux_wire(cell: &str, bit: Bit, input: &str) -> Result<u32, AdapterError> {
    let Bit::Wire(wire) = bit else {
        return Err(AdapterError::InvalidWideLut {
            cell: cell.into(),
            reason: format!("{input} is tied to a constant"),
        });
    };
    Ok(wire)
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
    /// A dedicated wide-LUT mux was not driven by the required LUT cascade.
    InvalidWideLut {
        /// Struo mux cell name.
        cell: String,
        /// Structural mismatch.
        reason: String,
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
            Self::InvalidWideLut { cell, reason } => {
                write!(f, "invalid wide-LUT cell {cell}: {reason}")
            }
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
            | Self::DisabledJtagOutput { .. }
            | Self::InvalidWideLut { .. } => None,
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
        EnableControl, MemoryCell, MemoryPort, MemoryStyle, Netlist, RegisterCell, ResetControl,
    };
    use struo_target_ecp5::{
        Bit, Ecp5Cell, IoTimingConstraints, JtaggBinding, MappingOptions, OpenDrainIo, PllBinding,
        PllOutput as StruoPllOutput, map_to_ecp5, map_to_ecp5_with_constraints,
        map_to_ecp5_with_jtagg, map_to_ecp5_with_open_drain_ios, map_to_ecp5_with_pll,
    };
    use texo_model::{CellId, ResourceKind};

    use super::{
        ActiveLevel, BlockRamPortMetadata, ClockEdge, DistributedRamRole, Importer, PortDirection,
        PrimitiveMetadata, ResetMetadata, celox_frontend_artifact, fold_lut_input, import_ecp5,
        pack_carry_inputs,
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
    fn expands_distributed_ram_into_one_atomic_physical_macro() {
        let mut source = Netlist::new("distributed_memory");
        let clock = source.add_input("clock");
        let write_enable = source.add_input("write_enable");
        let read_address = (0..4)
            .map(|index| source.add_input(format!("read_address_{index}")))
            .collect::<Vec<_>>();
        let write_address = (0..4)
            .map(|index| source.add_input(format!("write_address_{index}")))
            .collect::<Vec<_>>();
        let write_data = (0..4)
            .map(|index| source.add_input(format!("write_data_{index}")))
            .collect::<Vec<_>>();
        let read_data = (0..4)
            .map(|index| source.add_memory_output(format!("read_data_{index}")))
            .collect::<Vec<_>>();
        source.add_memory(
            MemoryCell::new(
                "words",
                16,
                read_address,
                read_data.clone(),
                None,
                write_address,
                write_data,
                EnableControl {
                    signal: write_enable,
                    active: StruoActiveLevel::Low,
                },
                clock,
                StruoClockEdge::Falling,
            )
            .with_style(MemoryStyle::Distributed)
            .with_read_latency(0),
        );
        for (index, output) in read_data.into_iter().enumerate() {
            source.add_output(format!("read_data_{index}"), output);
        }

        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();
        let [cluster] = imported.distributed_ram_clusters() else {
            panic!("expected one distributed-RAM cluster")
        };
        assert_eq!(
            cluster
                .data
                .into_iter()
                .chain(cluster.blockers)
                .chain([cluster.write_port])
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            7
        );
        assert_eq!(
            imported.metadata()[&cluster.data[0]],
            PrimitiveMetadata::DistributedRam {
                role: DistributedRamRole::Data(0),
                edge: ClockEdge::Falling,
                write_enable: ActiveLevel::Low,
            }
        );
        assert!(matches!(
            imported.metadata()[&cluster.write_port],
            PrimitiveMetadata::DistributedRam {
                role: DistributedRamRole::WritePort,
                ..
            }
        ));
        assert_eq!(
            imported.design().cells()[cluster.write_port.0].pins().len(),
            16
        );
        assert!(cluster.data.into_iter().all(|cell| {
            imported.design().cells()[cell.0].pins().len() == 12
                && imported.design().cells()[cell.0].kind == ResourceKind::Lut(4)
        }));
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
        source.add_input("clk_125");
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
            .additional_output_clock_ports
            .insert("clk_125".into(), StruoPllOutput::Clkos2);
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
                ("CLKOS2", texo_model::PinDirection::Output),
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
    fn absorbs_pfumx_and_l6mux21_into_one_four_lut_cluster() {
        let mut source = Netlist::new("six_input_parity");
        let inputs = source.add_input_port("inputs", NonZeroU32::new(6).unwrap());
        let parity = inputs[1..]
            .iter()
            .fold(inputs[0], |value, input| source.add_xor(value, *input));
        source.add_output("result", parity);
        let constraints = IoTimingConstraints::new()
            .with_input_delay_ps("inputs", 0)
            .with_output_delay_ps("result", 0);
        let mapped = map_to_ecp5_with_constraints(
            &source,
            MappingOptions {
                timing_goal_mhz: 1_500,
                ..MappingOptions::default()
            },
            &constraints,
        )
        .unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::PfuMux { .. }))
                .count(),
            2
        );
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::L6Mux21 { .. }))
                .count(),
            1
        );

        let imported = import_ecp5(&mapped).unwrap();

        let [cluster] = imported.wide_lut_clusters() else {
            panic!("expected one LUT6 cluster")
        };
        assert_eq!(cluster.len(), 4);
        let pin_names = cluster
            .iter()
            .map(|cell| {
                imported.design().cells()[cell.0]
                    .pins()
                    .iter()
                    .map(|pin| imported.design().pins()[pin.0].name.as_str())
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        assert!(pin_names[0].ends_with(&["F1", "M", "OFX"]));
        assert!(pin_names[1].ends_with(&["FXA", "FXB", "M", "OFX"]));
        assert!(pin_names[2].ends_with(&["F1", "M", "OFX"]));
        assert_eq!(&pin_names[3][..5], &["A", "B", "C", "D", "F"]);
    }

    #[test]
    fn absorbs_nested_l6mux21s_into_one_eight_lut_cluster() {
        let mut source = Netlist::new("seven_input_parity");
        let inputs = source.add_input_port("inputs", NonZeroU32::new(7).unwrap());
        let parity = inputs[1..]
            .iter()
            .fold(inputs[0], |value, input| source.add_xor(value, *input));
        source.add_output("result", parity);
        let constraints = IoTimingConstraints::new()
            .with_input_delay_ps("inputs", 0)
            .with_output_delay_ps("result", 0);
        let mapped = map_to_ecp5_with_constraints(
            &source,
            MappingOptions {
                timing_goal_mhz: 1_500,
                ..MappingOptions::default()
            },
            &constraints,
        )
        .unwrap();
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::PfuMux { .. }))
                .count(),
            4
        );
        assert_eq!(
            mapped
                .cells()
                .iter()
                .filter(|cell| matches!(cell, Ecp5Cell::L6Mux21 { .. }))
                .count(),
            3
        );

        let imported = import_ecp5(&mapped).unwrap();

        let [cluster] = imported.wide_lut_clusters() else {
            panic!("expected one LUT7 cluster")
        };
        assert_eq!(cluster.len(), 8);
        let pin_names = cluster
            .iter()
            .map(|cell| {
                imported.design().cells()[cell.0]
                    .pins()
                    .iter()
                    .map(|pin| imported.design().pins()[pin.0].name.as_str())
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        for &index in &[0, 2, 4, 6] {
            assert!(pin_names[index].ends_with(&["F1", "M", "OFX"]));
        }
        for &index in &[1, 3, 5] {
            assert!(pin_names[index].ends_with(&["FXA", "FXB", "M", "OFX"]));
        }
        assert_eq!(&pin_names[7][..5], &["A", "B", "C", "D", "F"]);
    }

    #[test]
    fn clones_a_pfumx_root_whose_f_output_has_other_fanout() {
        let root = Ecp5Cell::Lut4 {
            name: "root".into(),
            inputs: [Bit::Wire(0), Bit::Wire(1), Bit::Zero, Bit::Zero],
            output: 10,
            init: 0x6996,
        };
        let child = Ecp5Cell::Lut4 {
            name: "child".into(),
            inputs: [Bit::Wire(2), Bit::Wire(3), Bit::Zero, Bit::Zero],
            output: 11,
            init: 0x9669,
        };
        let consumer = Ecp5Cell::Lut4 {
            name: "consumer".into(),
            inputs: [Bit::Wire(10), Bit::Zero, Bit::Zero, Bit::Zero],
            output: 13,
            init: 0xaaaa,
        };
        let pfu = Ecp5Cell::PfuMux {
            name: "pfu".into(),
            lut_true: Bit::Wire(11),
            lut_false: Bit::Wire(10),
            select: Bit::Wire(4),
            output: 12,
        };
        let mut importer = Importer::new();
        importer.add_primitive(&root).unwrap();
        importer.add_primitive(&child).unwrap();
        importer.add_primitive(&consumer).unwrap();
        importer.add_pfu_mux(&pfu).unwrap();

        let [clone, child]: [CellId; 2] =
            importer.pending_wide_muxes[&12].clone().try_into().unwrap();
        assert_eq!(child, CellId(1));
        assert_ne!(clone, CellId(0));
        assert_eq!(importer.metadata[&clone], importer.metadata[&CellId(0)]);
        assert!(
            importer.design.cells()[CellId(0).0]
                .pins()
                .iter()
                .any(|pin| importer.design.pins()[pin.0].name == "F")
        );
        let clone_pins = importer.design.cells()[clone.0]
            .pins()
            .iter()
            .map(|pin| importer.design.pins()[pin.0].name.as_str())
            .collect::<Vec<_>>();
        assert!(clone_pins.ends_with(&["F1", "M", "OFX"]));
        assert!(!clone_pins.contains(&"F"));
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
    fn inserts_a_ccu2c_feedin_for_a_routed_carry_input() {
        let mut source = Netlist::new("carry_input");
        let width = NonZeroU32::new(8).unwrap();
        let lhs = source.add_input_port("lhs", width);
        let rhs = source.add_input_port("rhs", width);
        let carry = source.add_input("carry");
        let sum = source.add_arithmetic_with_carry(&lhs, &rhs, carry).unwrap();
        source.add_output_port("sum", &sum).unwrap();

        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();

        assert_eq!(imported.carry_pairs().len(), 6);
        let feedin = &imported.carry_pairs()[4];
        assert_eq!(
            imported.design().cells()[feedin[0].0].name,
            "$carry_feedin0$slice0"
        );
        let input = imported.design().cells()[feedin[0].0]
            .pins()
            .iter()
            .copied()
            .find(|pin| imported.design().pins()[pin.0].name == "A")
            .expect("carry feed-in must enter through a routable LUT input");
        let net = imported.design().pins()[input.0]
            .net()
            .expect("carry feed-in input must be connected");
        let driver = imported.design().nets()[net.0].driver;
        let driver_cell = imported.design().pins()[driver.0].cell;
        assert!(matches!(
            imported.metadata()[&driver_cell],
            PrimitiveMetadata::Port { ref name, .. } if name == "carry"
        ));
        for net in imported.design().nets() {
            if net
                .sinks
                .iter()
                .any(|sink| imported.design().pins()[sink.0].name == "FCI")
            {
                assert_eq!(
                    imported.design().pins()[net.driver.0].name,
                    "FCO",
                    "every dedicated FCI edge must originate at FCO"
                );
            }
        }
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

    #[test]
    #[allow(clippy::too_many_lines)]
    fn imports_true_dual_port_dp16kd_pins_and_edges() {
        let mut source = Netlist::new("true_dual_memory");
        let clock_a = source.add_input("clock_a");
        let clock_b = source.add_input("clock_b");
        let write_enable_a = source.add_input("write_enable_a");
        let write_enable_b = source.add_input("write_enable_b");
        let read_enable_a = source.add_input("read_enable_a");
        let read_enable_b = source.add_input("read_enable_b");
        let address_a = (0..2)
            .map(|index| source.add_input(format!("address_a_{index}")))
            .collect::<Vec<_>>();
        let address_b = (0..2)
            .map(|index| source.add_input(format!("address_b_{index}")))
            .collect::<Vec<_>>();
        let write_data_a = (0..2)
            .map(|index| source.add_input(format!("write_data_a_{index}")))
            .collect::<Vec<_>>();
        let write_data_b = (0..2)
            .map(|index| source.add_input(format!("write_data_b_{index}")))
            .collect::<Vec<_>>();
        let read_data_a = (0..2)
            .map(|index| source.add_memory_output(format!("read_data_a_{index}")))
            .collect::<Vec<_>>();
        let read_data_b = (0..2)
            .map(|index| source.add_memory_output(format!("read_data_b_{index}")))
            .collect::<Vec<_>>();
        source.add_memory(
            MemoryCell::new(
                "words",
                4,
                address_a.clone(),
                read_data_a.clone(),
                Some(EnableControl {
                    signal: read_enable_a,
                    active: StruoActiveLevel::High,
                }),
                address_a,
                write_data_a,
                EnableControl {
                    signal: write_enable_a,
                    active: StruoActiveLevel::High,
                },
                clock_a,
                StruoClockEdge::Falling,
            )
            .with_second_port(MemoryPort::new(
                address_b.clone(),
                read_data_b.clone(),
                Some(EnableControl {
                    signal: read_enable_b,
                    active: StruoActiveLevel::Low,
                }),
                address_b,
                write_data_b,
                EnableControl {
                    signal: write_enable_b,
                    active: StruoActiveLevel::Low,
                },
                clock_b,
                StruoClockEdge::Rising,
            )),
        );
        for (index, output) in read_data_a.into_iter().enumerate() {
            source.add_output(format!("read_data_a_{index}"), output);
        }
        for (index, output) in read_data_b.into_iter().enumerate() {
            source.add_output(format!("read_data_b_{index}"), output);
        }

        let imported = import_ecp5(&map_to_ecp5(&source).unwrap()).unwrap();
        let block_ram = imported
            .design()
            .cells()
            .iter()
            .position(|cell| cell.kind == ResourceKind::Memory)
            .map(CellId)
            .unwrap();
        let pin_names = imported.design().cells()[block_ram.0]
            .pins()
            .iter()
            .map(|pin| imported.design().pins()[pin.0].name.as_str())
            .collect::<Vec<_>>();

        assert!(pin_names.iter().any(|name| name.starts_with("DOA")));
        assert!(pin_names.iter().any(|name| name.starts_with("DOB")));
        assert!(pin_names.contains(&"CLKA"));
        assert!(pin_names.contains(&"CLKB"));
        assert!(pin_names.contains(&"CEA"));
        assert!(pin_names.contains(&"CEB"));
        assert!(pin_names.contains(&"WEA"));
        assert!(pin_names.contains(&"WEB"));
        assert_eq!(
            imported.metadata()[&block_ram],
            PrimitiveMetadata::BlockRam {
                depth: 4,
                word_width: 2,
                physical_width: 2,
                edge: ClockEdge::Falling,
                write_enable: ActiveLevel::High,
                read_enable: Some(ActiveLevel::High),
                second_port: Some(BlockRamPortMetadata {
                    edge: ClockEdge::Rising,
                    write_enable: ActiveLevel::Low,
                    read_enable: Some(ActiveLevel::Low),
                }),
            }
        );
    }
}
