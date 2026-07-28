# Rust Crate API Reference

These links open the Rustdoc generated for the host and STM32 crates by `./build_docs_local.sh`; they return 404 in the high-level-only `.venv/bin/zensical serve` preview.

---

<div class="grid cards" style="display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 1rem; margin-top: 1.5rem;">
  <div class="card" style="border: 1px solid var(--md-accent-fg-color); padding: 1.5rem; border-radius: 8px; background: rgba(30,41,59,0.2); transition: transform 0.2s;">
    <h3 style="margin-top:0;"><a href="veloxity_core/index.html" style="text-decoration:none; color:var(--md-accent-fg-color);">veloxity_core</a></h3>
    <p>Platform-agnostic flight controller logic, sensors, and estimators.</p>
    <span style="font-size:0.8rem; padding:2px 6px; background:#334155; border-radius:4px;">Core Crate</span>
  </div>

  <div class="card" style="border: 1px solid var(--md-accent-fg-color); padding: 1.5rem; border-radius: 8px; background: rgba(30,41,59,0.2); transition: transform 0.2s;">
    <h3 style="margin-top:0;"><a href="stm_32/index.html" style="text-decoration:none; color:var(--md-accent-fg-color);">stm_32</a></h3>
    <p>Embassy-based STM32 hardware abstraction layer and peripheral drivers.</p>
    <span style="font-size:0.8rem; padding:2px 6px; background:#334155; border-radius:4px;">STM32 HAL</span>
  </div>

  <div class="card" style="border: 1px solid var(--md-accent-fg-color); padding: 1.5rem; border-radius: 8px; background: rgba(30,41,59,0.2); transition: transform 0.2s;">
    <h3 style="margin-top:0;"><a href="pixracerpro/index.html" style="text-decoration:none; color:var(--md-accent-fg-color);">pixracerpro</a></h3>
    <p>Flight controller firmware implementation for the Pixracer Pro autopilot.</p>
    <span style="font-size:0.8rem; padding:2px 6px; background:#334155; border-radius:4px;">3DR/mRo Target</span>
  </div>

  <div class="card" style="border: 1px solid var(--md-accent-fg-color); padding: 1.5rem; border-radius: 8px; background: rgba(30,41,59,0.2); transition: transform 0.2s;">
    <h3 style="margin-top:0;"><a href="nucleo/index.html" style="text-decoration:none; color:var(--md-accent-fg-color);">nucleo</a></h3>
    <p>Firmware implementation optimized for the STM32H753 Nucleo-144 board.</p>
    <span style="font-size:0.8rem; padding:2px 6px; background:#334155; border-radius:4px;">Nucleo Target</span>
  </div>

  <div class="card" style="border: 1px solid var(--md-accent-fg-color); padding: 1.5rem; border-radius: 8px; background: rgba(30,41,59,0.2); transition: transform 0.2s;">
    <h3 style="margin-top:0;"><a href="sim/index.html" style="text-decoration:none; color:var(--md-accent-fg-color);">sim</a></h3>
    <p>Software-in-the-loop (SIL) simulation framework and RMw compatibility layer.</p>
    <span style="font-size:0.8rem; padding:2px 6px; background:#334155; border-radius:4px;">SIL Simulator</span>
  </div>
</div>

---

!!! tip

    Use the search bar in the top right of this portal to search high-level
    documentation. Low-level API documentation search can be performed locally
    inside the crate API pages themselves.
