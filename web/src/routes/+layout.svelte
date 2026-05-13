<script lang="ts">
  let { children } = $props();
</script>

{@render children()}

<style>
  :global(:root) {
    --primary: #5b8db8;
    --primary-dark: #4a7a9e;
    --primary-subtle: rgba(91, 141, 184, 0.1);
    --accent: #f5a623;
    --section-orange: #FC4C02;
    --bg: #fafafa;
    --bg-elevated: #ffffff;
    --card: #ffffff;
    --card-hover: #f0f1f3;
    --text: #1a1a1a;
    --text-muted: #666666;
    --text-dim: #999999;
    --border: #e0e0e0;
    --border-subtle: #f0f0f0;
    --highlight: rgba(91, 141, 184, 0.08);
    --radius: 12px;
    --radius-lg: 20px;
    --shadow: 0 1px 3px rgba(0,0,0,0.06);
    --shadow-lg: 0 4px 12px rgba(0,0,0,0.08);
    --sport-ride: #3B82F6;
    --sport-run: #10B981;
    --sport-walk: #8B5CF6;
    --sport-swim: #06B6D4;
    --sport-other: #64748b;
  }

  :global(:root[data-theme="dark"]) {
    --bg: #0d1117;
    --bg-elevated: #161b22;
    --card: #161b22;
    --card-hover: #1c2129;
    --text: #e6edf3;
    --text-muted: #8b949e;
    --text-dim: #6e7681;
    --border: #30363d;
    --border-subtle: #21262d;
    --highlight: rgba(91, 141, 184, 0.15);
    --shadow: 0 1px 3px rgba(0,0,0,0.3);
    --shadow-lg: 0 4px 12px rgba(0,0,0,0.4);
  }

  /* Fallback for before JS runs */
  @media (prefers-color-scheme: dark) {
    :global(:root:not([data-theme])) {
      --bg: #0d1117;
      --bg-elevated: #161b22;
      --card: #161b22;
      --card-hover: #1c2129;
      --text: #e6edf3;
      --text-muted: #8b949e;
      --text-dim: #6e7681;
      --border: #30363d;
      --border-subtle: #21262d;
      --highlight: rgba(91, 141, 184, 0.15);
      --shadow: 0 1px 3px rgba(0,0,0,0.3);
      --shadow-lg: 0 4px 12px rgba(0,0,0,0.4);
    }
  }

  :global(body) {
    margin: 0;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
    background: var(--bg);
    color: var(--text);
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
  }
</style>
