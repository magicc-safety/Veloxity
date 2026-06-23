# Documentation

This project uses a blended documentation system powered by **Zensical** (a
high-performance Rust-core static site generator) and Cargo API reference docs.

## Building and Merging the Docs

You can compile all API crate targets (embedded & simulator) and build the
high-level Zensical documentation tree with a single script:

```bash
./build_docs_local.sh
```

This script will automatically bootstrap a Python virtual environment (`.venv`)
using `uv`, install `zensical`, compile the cargo API documentation targets,
compile the Zensical guides, and merge them under the `./site` directory.

## Hosting the Documentation Locally

To host the compiled documentation locally and preview it in your browser:

1. Run a local HTTP server pointing to the output `site` directory:
   ```bash
   python3 -m http.server 8000 --directory site
   ```
2. Open your browser and navigate to `http://localhost:8000`.

## Active Development Preview

For instant previewing of high-level guides during editing (without API
merging):

```bash
.venv/bin/zensical serve
```
