import socket

def send_command(cmd):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('localhost', 8888))
            s.sendall(cmd.encode())
            return s.recv(1024)
    except ConnectionRefusedError:
        print("Error: Could not connect to server. Is connect_to_unity.py running?")
        return None

def main():
    print("Movement Controls")
    print("w - Forward | a - Left | s - Back | d - Right")
    
    try:
        while True:
            cmd = input("Command: ").lower()
            if cmd in ['w', 'a', 's', 'd']:
                response = send_command(cmd)
                if response != b"OK":
                    print("Movement failed!")
            else:
                print("Invalid command!")
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()