<script lang="ts">
	let { children } = $props();
</script>

{@render children()}

<style>
	:global(:root) {
		--primary: #5b8db8;
		--primary-dark: #4a7a9e;
		--accent: #f5a623;
		--section-orange: #FC4C02;
		--bg: #fafafa;
		--card: #ffffff;
		--text: #1a1a1a;
		--text-muted: #666666;
		--border: #e0e0e0;
		--radius: 12px;
		--radius-lg: 20px;
	}

	@media (prefers-color-scheme: dark) {
		:global(:root) {
			--bg: #121212;
			--card: #1e1e1e;
			--text: #ffffff;
			--text-muted: #aaaaaa;
			--border: #333333;
		}
	}

	:global(body) {
		margin: 0;
		font-family:
			-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
		background: var(--bg);
		color: var(--text);
		-webkit-font-smoothing: antialiased;
		-moz-osx-font-smoothing: grayscale;
	}
</style>
