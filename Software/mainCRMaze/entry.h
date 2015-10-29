struct entry
{
	uint8_t distance;
	uint8_t walls; // |**= 6(0110b=NW), |__= 5 (0101b=SW) , **|= 10 (1010b=NE), __|= 9 (1001b=SE), |.= 7 (0111b=W),  .|= 11 (1011b=E), ._.= 13 (1101b=S), .*.= 14 (1110b=N),  =15 (sin paredes)  //W=8,E=4,S=2,N=1

};
