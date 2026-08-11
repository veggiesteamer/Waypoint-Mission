from user_data.config_map import scenarios_config
from user_data.config_map import baseline_config
from src.operation import scenarios_march_review
import json
import pprint
from pathlib import Path

OUTPUT_DIR = Path("experiments/scenario_comparison")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


scenarios = {
    "nominal": {},

    "sparse_craters": {
        "k_crater": 0.0084,
        "keep_out_crater": 0
    },

    "dense_craters": {
        "keep_out_crater": 1
    }
}

results = {}

for name, override in scenarios.items():
    config = scenarios_march_review.merge_config(baseline_config.config, override)
    #results[name] = scenarios_march_review.run_simulation(config)
    results[name] = scenarios_march_review.run_experiment(config,n_runs=baseline_config.config["num_runs"])

#results = sensitivity_march_review.run_parameter_sweep(baseline_config.config,parameter_name="keep_out_crater",values=values,n_runs=baseline_config.config["num_runs"])
#pprint.pprint(results, sort_dicts=False, width=200)
with open(OUTPUT_DIR / "experiments_scenarios.json", "w") as f:
    json.dump(results, f)
    f.write("\n")

scenarios_march_review.print_summary_table(results)
scenarios_march_review.plot_success_rate(results)
scenarios_march_review.plot_path_metrics(results)
scenarios_march_review.plot_relative_to_nominal(results)

import pandas as pd
import matplotlib.pyplot as plt

df = pd.DataFrame(scenarios_march_review.flatten_data(results))

plt.figure()
plt.scatter(
    df["k_crater"],
    df["keep_out_crater"],
    s=df["tortuosity_factor_mean"],  # bubble size
)
plt.xlabel("Crater Count")
plt.ylabel("Boulder Density Factor")
plt.title("Environment vs Path Length")
plt.show()

success = df[df["success_rate"] > 0]
failure = df[df["success_rate"] == 0]
plt.figure()
plt.scatter(success["k_boulder"], success["num_craters"], label="Success")
plt.scatter(failure["k_boulder"], failure["num_craters"], label="Failure")

plt.xlabel("k_boulder")
plt.ylabel("num_craters")
plt.legend()
plt.title("Feasibility Boundary")
plt.show()