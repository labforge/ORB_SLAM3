import psutil
import sys
import time
import csv
import signal

# Flag to track if Ctrl+C signal was received
exit_flag = False

def handle_signal(signal, frame):
    global exit_flag
    exit_flag = True
    print("Ctrl+C signal received. Exiting...")

def monitor_process(pid):
    process = psutil.Process(pid)
    process_name = process.name()
    print(f"Monitoring Process: {process_name} (PID: {pid})")

    # Open the CSV file for writing
    with open('process_monitor.csv', 'w', newline='') as csvfile:
        fieldnames = ['Timestamp', 'CPU Usage (Absolute)', 'CPU Usage (Percentage)', 'Memory Usage (Absolute)', 'Memory Usage (Percentage)']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write the header row
        writer.writeheader()

        cpu_absolute_sum = 0.0
        iteration_count = 0

        process_cpu_absolute_avg = 0.0
        process_cpu_percent_avg = 0.0
        process_memory_absolute_avg = 0.0
        process_memory_percent_avg = 0.0

        process_cpu_absolute_sum = 0.0
        process_cpu_percent_sum = 0.0
        process_memory_absolute_sum = 0.0
        process_memory_percent_sum = 0.0

        while process.is_running() and not exit_flag:
            try:
                process_cpu_percent = process.cpu_percent()
                process_cpu_times = process.cpu_times()
                process_cpu_absolute = sum(process_cpu_times)
                process_memory_info = process.memory_info()
                process_memory_absolute = process_memory_info.rss
                process_memory_percent = (process_memory_absolute / psutil.virtual_memory().total) * 100

                # Get the current timestamp
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

		# Accumulate the sum of process_cpu_absolute and increment the count
                cpu_absolute_sum += process_cpu_absolute

                process_cpu_absolute_sum += process_cpu_absolute
                process_cpu_percent_sum += process_cpu_percent
                process_memory_absolute_sum += process_memory_absolute
                process_memory_percent_sum += process_memory_percent 
                iteration_count += 1

		# Calculate the average of process_cpu_absolute
                process_cpu_absolute_avg = process_cpu_absolute_sum/iteration_count
                process_cpu_percent_avg = process_cpu_percent_sum/iteration_count
                process_memory_absolute_avg = process_memory_absolute_sum/iteration_count
                process_memory_percent_avg = process_memory_percent_sum/iteration_count

                # Print and save the values to the CSV file
                print(f"Process CPU Usage: Absolute={process_cpu_absolute:.2f} seconds, Percentage={process_cpu_percent:.2f}%")
                print(f"Process Memory Usage: Absolute={process_memory_absolute} bytes, Percentage={process_memory_percent:.2f}%")
                writer.writerow({'Timestamp': timestamp,
                                 'CPU Usage (Absolute)': process_cpu_absolute,
                                 'CPU Usage (Percentage)': process_cpu_percent,
                                 'Memory Usage (Absolute)': process_memory_absolute,
                                 'Memory Usage (Percentage)': process_memory_percent})

                print("-" * 50)
                sys.stdout.flush()
                time.sleep(1)

            except psutil.NoSuchProcess:
                print("Process terminated.")
                break

        # Write the average values to the CSV file
        writer.writerow({'Timestamp': 'Average',
                         'CPU Usage (Absolute)': process_cpu_absolute_avg,
                         'CPU Usage (Percentage)': process_cpu_percent_avg,
                         'Memory Usage (Absolute)': process_memory_absolute_avg,
                         'Memory Usage (Percentage)': process_memory_percent_avg})

        print(f"Process CPU Usage Average: Absolute Average={process_cpu_absolute_avg:.2f} seconds, Percentage Average={process_cpu_percent_avg:.2f}%")
        print(f"Process Memory Usage Average: Absolute Average={process_memory_absolute_avg} bytes, Percentage Average={process_memory_percent_avg:.2f}%")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python monitor.py <PID>")
        sys.exit(1)

    pid = int(sys.argv[1])

    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, handle_signal)

    monitor_process(pid)

