import socket
import select

import time
import random

def noisy_broadcast_message(sender_socket, message, client_sockets):
    """Sends a message to all clients except the sender, simulating network issues."""
    
    # --- Simulate Network Imperfections ---
    # 1. Simulate Packet Loss (e.g., 1% chance to drop the message)
    if random.random() < 0.01:
        print(f"--- PACKET DROPPED ---")
        return

    # 2. Simulate Latency and Jitter (e.g., 50ms to 150ms delay)
    delay = random.uniform(0.05, 0.15)
    time.sleep(delay)
    
    # --- Broadcast to other clients ---
    for client_socket in client_sockets:
        if client_socket != sender_socket:
            try:
                client_socket.send(message)
            except:
                client_socket.close()
                client_sockets.remove(client_socket)

def broadcast_message(sender_socket, message, client_sockets):
    """Sends a message to all clients except the sender."""
    for client_socket in client_sockets:
        if client_socket != sender_socket:
            try:
                client_socket.send(message)
            except:
                client_socket.close()
                client_sockets.remove(client_socket)

def run_hub():
    """Runs the main repeater hub server."""
    host = '0.0.0.0'
    port = 54321
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setblocking(False)
    server_socket.bind((host, port))
    server_socket.listen(10)
    
    client_sockets = [server_socket]
    print(f"Repeater hub listening on port {port}...")

    try:
        while True:
            read_sockets, _, _ = select.select(client_sockets, [], [])
            
            for sock in read_sockets:
                if sock == server_socket:
                    conn, addr = server_socket.accept()
                    client_sockets.append(conn)
                    print(f"New connection from {addr}")
                else:
                    try:
                        data = sock.recv(1024)
                        if data:
                            print(f"Relaying message: {data.decode().strip()}")
                            broadcast_message(sock, data, client_sockets)
                        else:
                            sock.close()
                            client_sockets.remove(sock)
                    except:
                        sock.close()
                        client_sockets.remove(sock)
    except KeyboardInterrupt:
        print("Shutting down hub.")
    finally:
        server_socket.close()

if __name__ == "__main__":
    run_hub()