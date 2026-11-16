# Compare two section dumps written by tests/section_dump.rs or
# tests/section_dump_geolife.rs, section by section. Each dump opens with a
# header naming its fold (digest, tree, corpus). Two dumps of one fold are not
# a comparison, so that case exits non-zero unless --same-fold is given.
# The ids are positional (sec_<sport>_<n>), so renumber the later dump to the
# earlier numbering when a section was dropped before comparing.
# Run: python3 -I tests/section_compare.py [--same-fold] before.jsonl after.jsonl
import json, sys, math, statistics
from collections import Counter
same_fold_ok = '--same-fold' in sys.argv[1:]
paths = [x for x in sys.argv[1:] if x != '--same-fold']
def is_header(r):
    return 'digest' in r and 'id' not in r
def read(path):
    lines = [json.loads(l) for l in open(path) if l.strip()]
    header = lines[0] if lines and is_header(lines[0]) else None
    return header, [r for r in lines if not is_header(r)]
hb, b = read(paths[0])
ha, a = read(paths[1])
def describe(name, h):
    if h is None:
        return f"{name}: no header (dump predates the header), digest unknown"
    return f"{name}: digest {h['digest']} tree {h['tree']} corpus {h['corpus']}"
one_fold = bool(hb and ha and hb['digest'] == ha['digest'])
if one_fold:
    print(f"one fold: both dumps carry digest {ha['digest']}, so this compares a fold with itself")
print(describe('before', hb))
print(describe('after', ha))
if one_fold and not same_fold_ok:
    print("refusing to compare; pass --same-fold to compare anyway")
    sys.exit(1)
bi = {r['id']: r for r in b}; ai = {r['id']: r for r in a}
print(f"before {len(b)} sections, after {len(a)} sections")
added = [r for r in a if r['id'] not in bi]; dropped = [r for r in b if r['id'] not in ai]
print("added ids:", [(r['id'], len(r['members']), round(r['distance_m'])) for r in added])
print("dropped ids:", [(r['id'], len(r['members']), r['visit_count'], round(r['distance_m']), r['rep']) for r in dropped])
# do dropped lines reappear under another id?
bl = {r['line_hash']: r['id'] for r in b}; al = {r['line_hash']: r['id'] for r in a}
print("dropped lines present under another id after:", [(r['id'], al.get(r['line_hash'])) for r in dropped])
print("added lines present under another id before:", [(r['id'], bl.get(r['line_hash'])) for r in added])
common_order = [r['id'] for r in b if r['id'] in ai] == [r['id'] for r in a if r['id'] in bi]
print("common ids in same order:", common_order)
print("min members before", min(len(r['members']) for r in b), "after", min(len(r['members']) for r in a))
print("sections under 2 members before", sum(len(r['members'])<2 for r in b), "after", sum(len(r['members'])<2 for r in a))
keys = ['line_hash','line_points','distance_m','rep','rep_range','rep_reversed','members','portions','visit_count','trace_ids','confidence','observation_count']
redrawn=[]; reversed_=[]; length=[]; member=[]; visit=[]; portion=[]; rep=[]; trace=[]; other=[]; same=0; changed=0
mem_added=0; mem_dropped=0; rows_added=0; rows_dropped=0
def hav(p,q):
    R=6371000; la1,lo1,la2,lo2=map(math.radians,(p[0],p[1],q[0],q[1]))
    d=math.sin((la2-la1)/2)**2+math.cos(la1)*math.cos(la2)*math.sin((lo2-lo1)/2)**2
    return 2*R*math.asin(math.sqrt(d))
for r in b:
    if r['id'] not in ai: continue
    s = ai[r['id']]
    diff = [k for k in keys if r[k]!=s[k]]
    if not diff: same+=1; continue
    changed+=1
    if r['line_hash']!=s['line_hash']:
        if r['polyline']==s['polyline'][::-1]: reversed_.append(r['id'])
        else: redrawn.append((r['id'], r['line_points'], s['line_points'], round(r['distance_m']), round(s['distance_m']), r['rep'], s['rep'], r['rep_range'], s['rep_range'], round(hav(r['first'],s['first']),1), round(hav(r['last'],s['last']),1)))
    if abs(r['distance_m']-s['distance_m'])>0.5: length.append((r['id'], round(r['distance_m']), round(s['distance_m'])))
    if r['rep']!=s['rep'] or r['rep_range']!=s['rep_range']: rep.append(r['id'])
    if r['members']!=s['members']:
        ad=sorted(set(s['members'])-set(r['members'])); dr=sorted(set(r['members'])-set(s['members']))
        member.append((r['id'], len(r['members']), len(s['members']), len(ad), len(dr))); mem_added+=len(ad); mem_dropped+=len(dr)
    if r['visit_count']!=s['visit_count']: visit.append((r['id'], r['visit_count'], s['visit_count']))
    if r['portions']!=s['portions']:
        portion.append(r['id']); bp=set(map(tuple,r['portions'])); ap=set(map(tuple,s['portions']))
        rows_added+=len(ap-bp); rows_dropped+=len(bp-ap)
    if r['trace_ids']!=s['trace_ids']: trace.append(r['id'])
    o=[k for k in diff if k in ('confidence','observation_count')]
    if o: other.append((r['id'], o))
print(f"\nkept sections unchanged: {same}; with any change: {changed} ({100*changed/len(b):.1f} % of before)")
print(f"lines reversed (same points, opposite direction): {len(reversed_)} {reversed_}")
print(f"lines redrawn (other than a reversal): {len(redrawn)}")
for x in redrawn: print("  ", x)
print(f"length changed (>0.5 m): {len(length)} {length}")
print(f"representative or range changed: {len(rep)} {rep}")
print(f"confidence/observation_count changed: {other}")
print(f"activity_traces keys changed: {len(trace)} {trace}")
print(f"membership changed: {len(member)} ({100*len(member)/len(b):.1f} %): rides join {mem_added}, leave {mem_dropped}")
only_add=[m for m in member if not m[4]]; only_drop=[m for m in member if not m[3]]; both=[m for m in member if m[3] and m[4]]
print(f"  sections only gaining {len(only_add)}, only losing {len(only_drop)}, both {len(both)}")
print("  largest moves:", sorted(member, key=lambda m: -(m[3]+m[4]))[:12])
up=[v for v in visit if v[2]>v[1]]; down=[v for v in visit if v[2]<v[1]]
print(f"visit counts changed: {len(visit)}: up {len(up)} (+{sum(v[2]-v[1] for v in up)}), down {len(down)} ({sum(v[2]-v[1] for v in down)})")
print(f"  total visits before {sum(r['visit_count'] for r in b)}, after {sum(r['visit_count'] for r in a)}")
print(f"portion rows changed: {len(portion)} sections; rows added {rows_added}, dropped {rows_dropped}")
memset=set(m[0] for m in member)
print(f"  rows moved with members unchanged: {len([p for p in portion if p not in memset])}")
# row shifts on kept members
so=eo=bo=0; shifts=[]
for id in portion:
    r,s=bi[id],ai[id]
    bp={}; ap={}
    for p in r['portions']: bp.setdefault(p[0],[]).append(p)
    for p in s['portions']: ap.setdefault(p[0],[]).append(p)
    for act in set(bp)&set(ap):
        for x,y in zip(bp[act],ap[act]):
            if x==y: continue
            ds=y[1]-x[1]; de=y[2]-x[2]
            if ds and de: bo+=1
            elif ds: so+=1
            elif de: eo+=1
            shifts.append((abs(ds),abs(de)))
ss=[x for x,_ in shifts if x]; es=[y for _,y in shifts if y]
print(f"  rows on kept members that moved: start only {so}, end only {eo}, both {bo}; |start| median {statistics.median(ss) if ss else 0} max {max(ss) if ss else 0}, |end| median {statistics.median(es) if es else 0} max {max(es) if es else 0}")
moved = len(added)+len(dropped)+len(redrawn)+len(reversed_)+len([l for l in length if l[0] not in set(x[0] for x in redrawn)])
print(f"\nbar: lines moved (added {len(added)} + dropped {len(dropped)} + redrawn {len(redrawn)} + reversed {len(reversed_)}; visits excluded) = {len(added)+len(dropped)+len(redrawn)+len(reversed_)} / {len(b)} = {100*(len(added)+len(dropped)+len(redrawn)+len(reversed_))/len(b):.1f} %")
print("\nPer-section table (changed only): id | members b->a (+joined/-left) | visits b->a | rows b->a | line | length")
for r in b:
    s=ai.get(r['id'])
    if s is None: print(f"  {r['id']:12} | {len(r['members']):3}-> -  | {r['visit_count']:3}-> -  | DROPPED | {r['distance_m']:.0f} m"); continue
    if all(r[k]==s[k] for k in keys): continue
    ma=len(set(s['members'])-set(r['members'])); md=len(set(r['members'])-set(s['members']))
    line = 'reversed' if r['id'] in reversed_ else ('redrawn' if r['line_hash']!=s['line_hash'] else 'same')
    print(f"  {r['id']:12} | {len(r['members']):3}->{len(s['members']):3} (+{ma}/-{md}) | {r['visit_count']:3}->{s['visit_count']:3} | {len(r['portions']):3}->{len(s['portions']):3} | {line:8} | {r['distance_m']:.0f}->{s['distance_m']:.0f} m")
