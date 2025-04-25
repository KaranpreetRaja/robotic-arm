import serial
import threading

def read_from_serial(ser):
    while True:
        try:
            line = ser.readline()
            if line:
                print(f"\n[DEVICE] {line.decode(errors='replace').strip()}")
                print("> ", end="", flush=True)  # Prompt again
        except Exception as e:
            print(f"\n[ERROR reading serial]: {e}")
            break

def main():
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print("Connected to /dev/ttyACM0 at 115200 baud.")
        print("Type something and press Enter to send. Ctrl+C to exit.\n")

        # Start a background thread to read serial data
        threading.Thread(target=read_from_serial, args=(ser,), daemon=True).start()

        while True:
            user_input = input("> ").strip()
            if user_input:
                ser.write((user_input + "\n").encode())

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
