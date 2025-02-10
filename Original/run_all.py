import statistics
import pandas as pd
from SeqMan import SeqMan

def process_tasks(file_path):
    tasks = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split(':')
            if len(parts) == 2:
                task_name = parts[0].strip()
                object_list = [obj.strip() for obj in parts[1].split(',')]
                tasks.append((task_name, object_list))
    
    results_data = []
    for task_name, object_list in tasks:
        runtimes = []
        solved_count = 0

        for i in range(10):  # Run the method 10 times
            print(f"{task_name} try {i}")
            solved, runtime = SeqMan(task_name, object_list, "random", True, True, True)
            if solved:
                solved_count += 1
                runtimes.append(runtime)
            print(f"{task_name} try {i} runtime {runtime}")
        if runtimes:
            mean_runtime = statistics.mean(runtimes)
            std_dev_runtime = statistics.stdev(runtimes) if len(runtimes) > 1 else 0.0
        else:
            mean_runtime = None
            std_dev_runtime = None
        
        solved_percentage = (solved_count / 10) * 100  # Calculate the percentage of solved runs
        results_data.append({
            "Task Name": task_name.split("/")[-1],
            "Mean Runtime (s)": mean_runtime,
            "Std Dev Runtime (s)": std_dev_runtime,
            "Solved (%)": solved_percentage
        })
    
    # Create a DataFrame
    df = pd.DataFrame(results_data)
    df.to_csv("run_output_Rand_PR.csv", sep=" ", index=False)

# Example usage
process_tasks('SeqMan-main/Orig/input.txt')