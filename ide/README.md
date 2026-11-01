# IDE Setup

Any editor with clangd support works.

## Prerequisites
- clangd installed in the container (included in Dockerfile)
- Container running: `docker-compose -f docker-compose.dev.yml up -d`
- Project built at least once to generate `compile_commands.json`

## Other IDEs

Contributions welcome. Please add a subdirectory here with configuration
files and a README explaining the setup process.
