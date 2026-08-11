from src.operation import sensitivity_march_review
from user_data.config_map import baseline_config
import json
from pathlib import Path

OUTPUT_DIR = Path("experiments/sensitivity_sweep")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

sweep_parameter = "Dmin_boulder"
values = [0.1, 0.2, 0.4, 0.6]
metrics = ["success_rate", "grid_occupation_mean", "tortuosity_factor_mean", "amount_turns_mean"]


results = sensitivity_march_review.run_parameter_sweep(baseline_config.config,parameter_name=sweep_parameter,values=values,n_runs=baseline_config.config["num_runs"])
sensitivity_march_review.plot_sensitivity(results,parameter=sweep_parameter,metrics=metrics, fig_title="Crater Keep out Sensitivity")

with open(OUTPUT_DIR / "experiments.json", "w") as f:
    json.dump(results, f)
    f.write("\n")