//
// Created by tic-tac on 10/19/18.
//



#include "Com/DataSocket.hpp"

DataSocket::DataSocket(const char *address_string, uint16_t server_port) {
	//Create socket
	struct sockaddr_in* server_address_p=&server_address;
	server_socket=socket(AF_INET, SOCK_STREAM, 0);
	if(server_socket<=0){
		perror("Error at socket creation");
		exit(EXIT_FAILURE);
	}
	//Address config
	memset(server_address_p, 0, sizeof(*server_address_p));
	server_address_p->sin_family=AF_INET;
	server_address_p->sin_port=htons(server_port);
	if(inet_pton(AF_INET, address_string, &server_address_p->sin_addr)<0){
		perror("Error at address conversion");
		exit(EXIT_FAILURE);
	}

	//Reusable addresses and ports
	int option_value=1;
	if(setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &option_value, sizeof(option_value))){
		perror("Error at setsockopt");
		exit(EXIT_FAILURE);
	}

	//Bind
	if(bind(server_socket,(struct sockaddr*)(server_address_p), sizeof(*server_address_p))){
		perror("Error at bind");
		exit(EXIT_FAILURE);
	}

	//Listen with queue of size 1
	if(listen(server_socket, 1)<0){
		perror("Error server listen");
		exit(EXIT_FAILURE);
	}
}

bool DataSocket::accept_client() {
	struct sockaddr_in* server_address_p=&server_address;
	int addr_len=sizeof(*server_address_p);
	int new_socket = accept(server_socket, (struct sockaddr *) server_address_p, (socklen_t *) &addr_len);
	if(new_socket<=0){
		perror("Error accept");
		return false;
	}
	client_socket=new_socket;
	return true;
}

int DataSocket::send_scan(data_wrappers::FullScan &scan) {
	ssize_t size=(15)*scan.size();
	char* send_buffer=new char[size];
	std::cout<<size<<" bytes allocated."<<std::endl;
	char* cursor=send_buffer;
	int c=0;
	for(data_wrappers::Measurement& measure : scan){
		cursor+=sprintf(cursor,"%d:%.2f;", measure.distance, measure.angle);
	}
	sprintf(cursor, "\n");
	printf("%d bytes written, %s",(int)(cursor-send_buffer),  send_buffer);
	int result=send(client_socket, send_buffer, strlen(send_buffer), 0);
	delete[] send_buffer;
	if(result<0){
		perror("Error sending data to client");
	}
	return result;
}

