# References

Scholarly sources behind tracematch's algorithm design. Code comments cite
these in short form (author or venue plus year). Add new entries here when a
design decision leans on published work.

## Trajectory similarity and matching

- Vlachos, Kollios, Gunopulos. Discovering similar multidimensional
  trajectories. ICDE 2002. LCSS with a spatial tolerance, the noise-robust
  measure behind partial-overlap portion matching.
- Trajectory similarity measures survey. ACM Computing Surveys, 2020.
  Two-stage matching: cheap prefilter, exact measure second. AMD stays as
  the prefilter.

## Section formation and boundaries

- Agarwal, Fox, Munagala, Nath, Pan, Taylor. Subtrajectory clustering:
  models and algorithms. PODS 2018. Facility-location objective: opening a
  section costs, coverage pays. Grounds the selection backoff.
- Panagiotakis, Pelekis, Kopanakis, Ramasso, Theodoridis. Segmentation and
  sampling of moving object trajectories based on representativeness.
  IEEE TKDE 2012. Representative real subtrajectories over synthetic
  averages.
- Fathi, Krumm. Detecting road intersections from GPS traces. GIScience
  2010. Corpus-derived decision points as boundary anchors.
- Newson, Krumm. Hidden Markov map matching through noise and sparseness.
  ACM SIGSPATIAL 2009. The graph-based alternative deliberately deferred:
  needs a routable map the app does not carry, and unmapped trails defeat
  it.

## Incremental dynamism

- Li et al. Incremental clustering for trajectories. DASFAA 2010.
  Micro-cluster lifecycle: birth, growth, merge.
- Schubert, Rousseeuw. Fast and eager k-medoids clustering. Information
  Systems 2021. Incremental representative update without full recompute.

## Interestingness: route choice as revealed preference

- Salazar Miranda, Fan, Duarte, Ratti. Desirable streets: using deviations
  in pedestrian trajectories to measure the value of the built environment.
  Computers, Environment and Urban Systems 86, 2021.
  https://doi.org/10.1016/j.compenvurbsys.2020.101563
  Desirability measured purely from GPS deviation against shortest paths.
  Desirable streets are sinuous and park-adjacent. Basis for the apex and
  sinuosity features.
- Quercia, Schifanella, Aiello. The shortest path to happiness:
  recommending beautiful, quiet, and happy routes in the city. ACM
  Hypertext 2014. https://arxiv.org/abs/1407.1031
  People prefer scenic paths about 12% longer and do not perceive the
  extra time.
- Urban physical environments promoting active leisure travel: an
  empirical study using crowdsourced GPS tracks and geographic big data
  from multiple sources. Land 13(5):589, 2024.
  https://doi.org/10.3390/land13050589
  Detour magnitudes by activity: running most detour-prone, cycling about
  2.8 times the beeline, hiking 4.3 km beeline against 10 km travelled.
- Why do bicyclists take detours? A multilevel regression model using
  smartphone GPS data. Journal of Transport Geography, 2018.
  https://doi.org/10.1016/j.jtrangeo.2018.04.012
  Recreational trips take detours utilitarian trips do not.
- How do cyclists make their way? A GPS-based revealed preference study in
  Copenhagen. International Journal of Geographical Information Science
  32(7), 2018. Gradient and greenery in revealed route choice.

## Interestingness: challenge, flow, attachment

- Measuring the motivation to ride bicycles for tourism through a
  comparison of tourist attractions. Transport Policy, 2017.
  Physical challenge, self-development, exploration and stimulus seeking
  as core cycling motives.
- When the mountains call: exploring mountaineering motivations through
  the lens of the calling theory. Journal of Outdoor Recreation and
  Tourism, 2024. Accomplishment and mastery drive repeated ascents.
- Health promotion as a motivational factor in alpine cycling.
  International Journal of Environmental Research and Public Health, 2021.
- Optimal experiences in exercise: a qualitative investigation of flow and
  clutch states. Psychology of Sport and Exercise, 2018.
- Optimal psychological states in advanced climbers: antecedents,
  characteristics, and consequences of flow and clutch states. Psychology
  of Sport and Exercise, 2022. Flow needs challenge-skill balance, clear
  goals, immediate feedback. Sections provide the goal-and-feedback
  scaffolding.
- Entropy measures can add novel information to reveal how runners' heart
  rate and speed are regulated by different environments. Frontiers in
  Psychology 10:1278, 2019. https://doi.org/10.3389/fpsyg.2019.01278
  Physiological regulation differs between usual and unusual routes on
  identical ground. Supports familiarity as a signal and rules physiology
  out as a boundary anchor.

## Carried-ground exclusion

- International Skyrunning Federation, Vertical Kilometre world record:
  1,000 m of gain in 29:42 (Fully, Switzerland, 2017). The all-time human
  vertical ceiling, ~2,020 m/h sustained for half an hour. Anchors the
  velocity veto's climb-rate floor: ground climbed well below this rate
  was climbed by a person, not ridden.
- EN 12929-1, Safety requirements for cableway installations designed to
  carry persons. Governs ropeway design including line speeds: fixed-grip
  chairlifts run ~2-2.5 m/s, detachable chairs and gondolas 4-6 m/s.
  Anchors the veto's ground-speed floor from the carried side.

## Climb characterisation

- Gradient-threshold climb segmentation convention: a climb is a sustained
  grade (roughly 3% or more) held over a few hundred metres, with short
  sub-threshold interruptions tolerated. Used for the sustained-gradient
  feature and auto-naming, never for boundary detection.
