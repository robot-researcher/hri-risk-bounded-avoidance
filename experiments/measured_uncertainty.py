"""Hardware-informed planar-model sensitivity and runtime-guard experiment."""
import argparse,json,hashlib,platform,time
from pathlib import Path
import numpy as np
from risk_bounded_collision.kinematics import PlanarArm3DOF
from risk_bounded_collision.primitives import generate_primitives
from risk_bounded_collision.risk import RiskCertifier,adaptive_risk_threshold
from risk_bounded_collision.selector import PrimitiveSelector
from risk_bounded_collision.runtime import SafetyRuntime,RuntimeObservation,SimulatedRobotAdapter

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--input',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); ap.add_argument('--link-lengths',type=float,nargs=3,default=(.4,.3,.2)); args=ap.parse_args()
    if args.output.exists(): raise ValueError('Choose a fresh output filename')
    source=json.loads(args.input.read_text()); ticks=np.array(source['samples_ticks'],float)
    if ticks.shape!=(30,3) or not np.isfinite(ticks).all(): raise ValueError('Expected 30 valid measured three-joint samples')
    rad_per_tick=source['radians_per_tick']; covariance=np.cov(ticks*rad_per_tick,rowvar=False,ddof=1)
    allowance=np.eye(3)*rad_per_tick**2/12; regularized=covariance+allowance
    arm=PlanarArm3DOF(tuple(args.link_lengths)); primitives=generate_primitives((0.,0.,0.),count=64)
    distances=tuple(float(x) for x in np.linspace(.005,.20,40)); goals=[primitives[i].terminal for i in (0,31,63)]
    rows=[]; summaries=[]; guards=[]
    for factor in (1,4,16):
        cov=regularized*factor; matrix=tuple(tuple(float(x) for x in row) for row in cov)
        begin=time.perf_counter(); library=RiskCertifier(arm).build_library(primitives,distances,matrix); build_s=time.perf_counter()-begin; selector=PrimitiveSelector(library)
        counts={'adaptive':0,'fixed':0}; guard_counts={'stale_hold':0,'wrong_start_hold':0,'checks_each':0}
        for distance in distances:
            threshold=adaptive_risk_threshold(distance,matrix)
            for gi,goal in enumerate(goals):
                for policy,delta in [('adaptive',threshold),('fixed',.05)]:
                    answer=selector.select(obstacle_distance_m=distance,threshold=delta,goal_joint_angles=goal)
                    if not answer.fallback and (answer.risk_bound is None or answer.risk_bound>delta): raise AssertionError('Risk-threshold invariant')
                    counts[policy]+=int(not answer.fallback)
                    rows.append(dict(covariance_multiplier=factor,distance_m=distance,goal_index=gi,policy=policy,threshold=delta,accepted=not answer.fallback,risk_bound=answer.risk_bound,primitive=answer.primitive_id))
                for kind,angles,age in [('stale_hold',(0.,0.,0.),1.),('wrong_start_hold',(.3,.3,.3),0.)]:
                    adapter=SimulatedRobotAdapter(); runtime=SafetyRuntime(selector,adapter)
                    decision=runtime.step(RuntimeObservation(angles,matrix,distance,age),goal)
                    passed=decision.command=='HOLD'; guard_counts[kind]+=int(passed)
                    guards.append(dict(covariance_multiplier=factor,distance_m=distance,goal_index=gi,case=kind,passed=passed,reason=decision.selection.reason))
                guard_counts['checks_each']+=1
        summaries.append(dict(covariance_multiplier=factor,synthetic_multiplier=factor!=1,trace_rad2=float(np.trace(cov)),offline_build_s=build_s,cases_per_policy=len(distances)*len(goals),accepted=counts,guards=guard_counts))
        print(json.dumps(summaries[-1]),flush=True)
    payload=dict(experiment='hardware-informed planar-model replay; no physical collision trials',python=platform.python_version(),platform=platform.platform(),input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),measured_covariance_rad2=covariance.tolist(),quantization_allowance_rad2=allowance.tolist(),link_lengths_m=arm.link_lengths_m,primitive_count=64,summaries=summaries,decisions=rows,guard_checks=guards,limitations=['Stationary sample covariance is not a verified distribution of tracking errors.','Factors 4 and 16 are synthetic covariance scaling (2x and 4x standard deviation).','The planar geometry is a research model, not calibrated JetArm geometry.','No physical collision probability or stopping distance is estimated.'])
    args.output.parent.mkdir(parents=True,exist_ok=True); args.output.write_text(json.dumps(payload,indent=2))

if __name__=='__main__': main()
