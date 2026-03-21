# IDE Setup

## Geany

The `topicfs.geany` project file is provided in the project root.

### Prerequisites
- Geany with the LSP Client plugin installed
- clangd installed in the container (included in Dockerfile)
- Container running: `podman-compose up -d`
- Project built at least once to generate `compile_commands.json`

### Setup
1. Open Geany
2. Open the workbench file: Tools → Workbench → Open
3. Select `smithjack.geanywb` — this file lives one level above the TopicFS
   repository root, in the same directory where you cloned the project
4. Double-click TopicFS in the Workbench tab

## Other IDEs

Contributions welcome. Please add a subdirectory here with configuration
files and a README explaining the setup process.
