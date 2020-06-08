from http.server import HTTPServer, BaseHTTPRequestHandler


class Server(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.path = "/websockets.htm"
            try:
                file_to_open = open(self.path[1:]).read()
                self.send_response(200)
            except:        
                file_to_open = "File not found"
                self.send_response(404)
            self.end_headers()    
            self.wfile.write(bytes(file_to_open, 'utf-8'))

def main():
    PORT=5000
    server_address =("localhost", 5000)
    server =HTTPServer(server_address, Server)
    print("Server in running on port", + PORT)
    server.serve_forever()
    
    
    


if __name__ == '__main__':
    main()