import threading
import time

print("Hello cruel world.")

def task(name):
    print(f"Thread {name}: starting")
    time.sleep(2)
    print(f"Thread {name}: finishing")

if __name__ == "__main__":
    threads = []
    for i in range(3):
        t = threading.Thread(target=task, args=(str(i),))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    print("All threads finished")
