<script lang="ts">
  let { mode = 'corridor' }: { mode: 'density' | 'flow' | 'corridor' } = $props();

  // Pseudo-traces on a 300x120 canvas. Eight traces representing
  // activities that share some corridors and diverge at others.
  const traces = [
    '20,100 60,80 120,60 180,50 240,50 280,60',
    '20,95 60,78 120,62 180,55 220,60 260,80 280,90',
    '20,105 55,85 110,65 170,52 230,48 280,55',
    '25,100 65,75 120,55 160,45 200,40 240,35 280,30',
    '25,105 60,82 115,68 155,48 195,42 235,38 275,35',
    '60,20 90,40 120,58 180,52 240,50 280,58',
    '65,25 95,42 120,55 150,50 180,48 210,55 240,70',
    '120,110 160,90 200,70 240,55 280,58',
  ];

  // Corridor: long dense stretches where many traces overlap
  const corridorHighlights = [
    '60,80 120,60 180,50 240,50',
    '20,100 60,80',
    '120,60 160,45 200,40 240,35 280,30',
  ];

  // Density grid: only where 3+ distinct route groups share a stretch
  const densityHighlights = [
    '100,62 140,55 180,50',
  ];

  // Flow graph: short edges between junction points
  const flowHighlights = [
    '60,80 90,68',
    '120,60 150,50',
    '180,50 210,50',
    '240,50 270,55',
    '120,60 145,48',
  ];

  const highlights = $derived(
    mode === 'corridor' ? corridorHighlights :
    mode === 'density' ? densityHighlights :
    flowHighlights
  );
</script>

<svg viewBox="0 0 300 120" class="method-illustration" xmlns="http://www.w3.org/2000/svg">
  {#each traces as points}
    <polyline {points} fill="none" stroke="rgba(255,255,255,0.12)" stroke-width="1.5" stroke-linecap="round" />
  {/each}
  {#each highlights as points}
    <polyline {points} fill="none" stroke="#FC4C02" stroke-width="3" stroke-linecap="round" opacity="0.9" />
  {/each}
</svg>

<style>
  .method-illustration {
    width: 100%;
    height: 120px;
    background: rgba(0,0,0,0.3);
    border-radius: 6px;
    margin: 8px 0;
  }
</style>
