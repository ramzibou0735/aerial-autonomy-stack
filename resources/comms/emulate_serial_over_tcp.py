import socket
import selectors
import random
import time

def broadcast_message(sender_key, message, sel, clients):
    # Create a list of recipients to handle disconnections safely
    recipients = list(clients.values())

    for client_key in recipients:
        if client_key == sender_key:
            continue
        try:
            client_key.fileobj.sendall(message)
        except (BrokenPipeError, ConnectionResetError):
            print(f"Client disconnected: {client_key.data.addr}")
            sel.unregister(client_key.fileobj)
            client_key.fileobj.close()
            del clients[client_key.data.addr]

# def noisy_broadcast_message(sender_key, message, sel, clients):
#     # Simulate packet loss (e.g., 1% chance to drop the message)
#     if random.random() < 0.01:
#         print(f"Drop packet")
#         return
#     # Simulate latency and jitter (e.g., 50ms to 150ms delay)
#     delay = random.uniform(0.05, 0.15)
#     time.sleep(delay)
#     broadcast_message(sender_key, message, sel, clients)

def accept_connection(sock, sel, clients):
    """Callback for new connections."""
    conn, addr = sock.accept()
    print(f"New connection from {addr}")
    conn.setblocking(False)
    # Store the address with the selector key
    key = sel.register(conn, selectors.EVENT_READ, data=type('Data', (), {'addr': addr}))
    clients[addr] = key

def handle_client_data(key, sel, clients):
    sock = key.fileobj
    try:
        data = sock.recv(1024)
        if data:
            # print(f"Relaying from {key.data.addr}: {data.decode().strip()}")
            broadcast_message(key, data, sel, clients)
            # noisy_broadcast_message(key, data, sel, clients)
            return len(data)
        else:
            # No data means the client has closed the connection
            print(f"Client closed connection: {key.data.addr}")
            sel.unregister(sock)
            sock.close()
            del clients[key.data.addr]
            return 0
    except (ConnectionResetError, BrokenPipeError):
        print(f"Client disconnected abruptly: {key.data.addr}")
        sel.unregister(sock)
        sock.close()
        del clients[key.data.addr]
        return 0

def main():
    host = '0.0.0.0'
    port = 54321
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(10)
    server_socket.setblocking(False)
    
    sel = selectors.DefaultSelector()
    sel.register(server_socket, selectors.EVENT_READ, data=None)
    
    clients = {}
    print(f"Repeater listening on port {port}...")

    bytes_received_interval = 0
    last_summary_time = time.time()
    summary_interval = 5  # In seconds

    try:
        while True:
            # Wait for an event (connection or data)
            events = sel.select(timeout=1.0) # Timeout so the loop runs even with no activity
            for key, mask in events:
                if key.data is None:
                    accept_connection(key.fileobj, sel, clients)
                else:
                    num_bytes = handle_client_data(key, sel, clients)
                    bytes_received_interval += num_bytes

            current_time = time.time()
            if current_time - last_summary_time > summary_interval:
                rate_kbps = (bytes_received_interval * 8) / (summary_interval * 1024)
                print(f"Traffic Summary - Clients: {len(clients)}, Rate: {rate_kbps:.2f} Kbps")
                bytes_received_interval = 0
                last_summary_time = current_time

    except KeyboardInterrupt:
        print("\nShutting down hub.")
    finally:
        sel.close()
        server_socket.close()

if __name__ == "__main__":
    main()
