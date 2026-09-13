"""Paired lookup versus online recomputation: identical model and decision rule."""
import argparse,hashlib,json,platform,time
from pathlib import Path
import numpy as np
from risk_bounded_collision.kinematics import PlanarArm3DOF
from risk_bounded_collision.primitives import generate_primitives
from risk_bounded_collision.risk import RiskCertifier,adaptive_risk_threshold
from risk_bounded_collision.selector import PrimitiveSelector

def stats(v):
    return dict(n=len(v),median_us=float(np.median(v)),p95_us=float(np.percentile(v,95)),p99_us=float(np.percentile(v,99)),max_us=float(max(v)))

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--input',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); ap.add_argument('--repeats',type=int,default=5); ap.add_argument('--link-lengths',type=float,nargs=3,default=(.4,.3,.2)); args=ap.parse_args()
    if args.output.exists(): raise ValueError('Choose a fresh output filename')
    src=json.loads(args.input.read_text()); ticks=np.array(src['samples_ticks'],float); scale=src['radians_per_tick']
    cov=np.cov(ticks*scale,rowvar=False,ddof=1)+np.eye(3)*scale**2/12
    matrix=tuple(tuple(float(x) for x in row) for row in cov)
    arm=PlanarArm3DOF(tuple(args.link_lengths)); primitives=generate_primitives((0.,0.,0.),count=64); bins=tuple(float(x) for x in np.linspace(.005,.20,40))
    certifier=RiskCertifier(arm); start=time.perf_counter(); library=certifier.build_library(primitives,bins,matrix); build=time.perf_counter()-start
    start=time.perf_counter(); selector=PrimitiveSelector(library,candidate_limit=16); index_build=time.perf_counter()-start
    def cached(distance,threshold,goal):
        r=selector.select(obstacle_distance_m=distance,threshold=threshold,goal_joint_angles=goal)
        return r.primitive_id,r.risk_bound
    def recompute(distance,threshold,goal):
        # Same conservative distance bin and candidate ranking as the lookup method.
        bin_index=max(0,min(int(np.searchsorted(bins,distance,side='right'))-1,len(bins)-1))
        ranked=sorted((certifier.continuous_swept_risk(p,bins[bin_index],matrix),i) for i,p in enumerate(primitives))
        candidates=[item for item in ranked if item[0]<=threshold][:16]
        if not candidates: return None,None
        risk,i=min(candidates,key=lambda item:sum((a-b)**2 for a,b in zip(primitives[item[1]].terminal,goal)))
        return primitives[i].primitive_id,risk
    queries=[(bins[i],adaptive_risk_threshold(bins[i],matrix),primitives[j].terminal) for i in (0,2,5,10,20,30,39) for j in (0,31,63)]
    for i in range(200): cached(*queries[i%len(queries)])
    recompute(*queries[-1])
    timings={'lookup':[],'online_recompute':[]}; rows=[]; mismatches=0
    args.output.parent.mkdir(parents=True,exist_ok=True)
    raw=args.output.with_suffix('.jsonl')
    with raw.open('w') as f:
        for rep in range(args.repeats):
            indices=np.random.RandomState(20260910+rep).permutation(len(queries))
            for pos,qi in enumerate(indices):
                outcomes={}; durations={}; order=('lookup','online_recompute') if (rep+pos)%2==0 else ('online_recompute','lookup')
                for method in order:
                    fn=cached if method=='lookup' else recompute
                    start=time.perf_counter(); outcomes[method]=fn(*queries[qi]); durations[method]=(time.perf_counter()-start)*1e6; timings[method].append(durations[method])
                a,b=outcomes['lookup'],outcomes['online_recompute']; equal=a[0]==b[0] and ((a[1] is None and b[1] is None) or (a[1] is not None and b[1] is not None and abs(a[1]-b[1])<1e-12))
                mismatches+=int(not equal); row=dict(repetition=rep,query_index=int(qi),distance_m=queries[qi][0],threshold=queries[qi][1],latency_us=durations,decisions=outcomes,equal=equal)
                f.write(json.dumps(row)+'\n'); f.flush(); rows.append(row)
            print(json.dumps(dict(repetition=rep,paired_cases=len(rows),decision_mismatches=mismatches,lookup=stats(timings['lookup']),online_recompute=stats(timings['online_recompute']))),flush=True)
    result=dict(experiment='paired embedded computation baseline, no arm commands',platform=platform.platform(),python=platform.python_version(),input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),covariance_rad2=cov.tolist(),link_lengths_m=list(arm.link_lengths_m),primitive_count=64,distance_bins=list(bins),candidate_limit=16,offline_library_build_s=build,selector_index_build_s=index_build,repetitions=args.repeats,paired_cases=len(rows),decision_mismatches=mismatches,timings={k:stats(v) for k,v in timings.items()},queries=[dict(distance_m=d,threshold=t,goal=list(g)) for d,t,g in queries],limitations=['Online recomputation is an uncached analytic baseline, not a competing planner implementation.','Both methods use the same risk expression, covariance, clearance bins and bounded candidate rule.','This tests caching speed and decision equivalence, not physical collision avoidance.','Synthetic clearance queries and stationary measured encoder covariance are used.'])
    result['median_latency_ratio']=result['timings']['online_recompute']['median_us']/result['timings']['lookup']['median_us']
    import inspect
    result['source_hashes']={cls.__name__:hashlib.sha256(Path(inspect.getfile(cls)).read_bytes()).hexdigest() for cls in (RiskCertifier,PrimitiveSelector,PlanarArm3DOF)}
    args.output.write_text(json.dumps(result,indent=2)); print('SAVED '+str(args.output),flush=True)

if __name__=='__main__': main()
