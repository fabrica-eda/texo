# Physical-design visualizer

Texo can turn a JSON implementation checkpoint into a self-contained HTML/SVG
view:

```sh
texo visualize path/to/checkpoint.json [path/to/output.html]
```

When the output argument is omitted, Texo writes
`path/to/checkpoint.json.html`. The result has no network, JavaScript package,
architecture-database, or web-server dependency; open it directly in a browser.

The main view contains the placed cells and one SVG path per routed net. A
route path is built from its checkpoint PIP endpoints, so it shows the actual
routing-resource topology at tile resolution. Fixed PIPs have a white dashed
overlay. Cell colors distinguish LUTs, flip-flops, carry logic, IO, and
constants. Logic cells sharing a tile are separated by their `SLICE` and
`K`/`FF` site names.

Controls:

- Drag to pan and use the wheel or trackpad to zoom. `Fit` restores the whole
  device view.
- Search by net name, cell name, BEL, or primitive kind. Press `/` to focus the
  search box.
- Click a route or cell to inspect its identity, location, and available
  setup/hold slack.
- Enable `timing <=` to retain only nets whose worst setup or hold slack is at
  or below the threshold in picoseconds.
- Toggle route, cell, and tile-grid layers independently. Press Escape to
  clear the selection.

The generated file embeds only visualization data: placement, primitive kind,
PIP endpoint coordinates, and reduced timing slack. It intentionally omits the
checkpoint's full timing-check and wire lists. For example, a 14 MiB AXI4
checkpoint with 2,379 cells and 21,822 PIPs produces an HTML file of about
1 MiB.
