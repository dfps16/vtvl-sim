import sys

from vtvl_sim.paths import result_path
from vtvl_sim.plotting import animate_descent, plot_engine, plot_propellant, plot_state, plot_trajectory
from vtvl_sim.post_processing import save_csv, write_sim_report
from vtvl_sim.scenario_io import load_scenario
from vtvl_sim.sim import sim_run

if __name__ == '__main__':
    scenario_path = sys.argv[1] if len(sys.argv) > 1 else 'test_scenarios/scenario1.json'

    sim_setup, solver_setup, outputs = load_scenario(scenario_path)
    sim_results = sim_run(sim_setup, solver_setup)

    if outputs['trajectory'] == 1:
        traj_plot = plot_trajectory(sim_results)
        traj_plot.savefig(result_path('last_sim_trajectory.png'), dpi=150)

    if outputs['state'] == 1:
        state_plot = plot_state(sim_results, sim_setup)
        state_plot.savefig(result_path('last_sim_state.png'), dpi=150)
        
    if outputs['engine'] == 1:
        engine_plot = plot_engine(sim_results, sim_setup)
        engine_plot.savefig(result_path('last_sim_engine.png'), dpi=150)

    if outputs['propellant'] == 1:
        propellant_plot = plot_propellant(sim_results, sim_setup)
        propellant_plot.savefig(result_path('last_sim_propellant.png'), dpi=150)

    if outputs['animation'] == 1:
        fig, anim, _ = animate_descent(
            sim_results, sim_setup,
            save_path=result_path('last_sim_animation.mp4'),
        )
    if outputs['report'] == 1:
        report = write_sim_report(sim_setup, sim_results)

    if outputs['csv'] == 1:
        csv = save_csv(sim_results)
