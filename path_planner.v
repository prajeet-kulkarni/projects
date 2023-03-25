module FPGA_path_planner
#(parameter node_count = 27, parameter max_edges = 4)
(
	input clk,
	input start,
	input [4:0] s_node,
	input [4:0] e_node,
	output reg done=0,	
	output reg [10*5-1:0] final_path
);
///////////////////////WRITE YOUR CODE FROM HERE////////////////////

reg[2:0]Y,y;
parameter [2:0]initialise=3'b000,INFINITY=5'B11111,distanceUpdate=3'b001,nextNodeUpdate=3'b010,finalise=3'b011,over=3'b100,waiting=3'b101,change=3'b111;
integer i,j,s,p,k,l,nextNode,nextNode1,eNode,van;
reg [4:0]dist[26:0][26:0];
reg [4:0]node[26:0];
reg [4:0]prevNode[26:0];
reg [4:0]strDist[26:0];
reg visited[26:0];
reg [4:0]deadend[12:0];
reg [4:0]min;
integer count=0;
reg[4:0]final_path1[9:0];
reg[4:0]intel;
reg flag;
parameter a= 5'd0;
				parameter b= 5'd1;
				parameter c= 5'd2;
				parameter d= 5'd3;
				parameter e= 5'd4;
				parameter f= 5'd5;
				parameter g= 5'd6;
				parameter h= 5'd7;
				parameter iam= 5'd8;
				parameter J= 5'd9;
				parameter K= 5'd10;
				parameter L= 5'd11;
				parameter m= 5'd12;
				parameter n= 5'd13;
				parameter o= 5'd14;
				parameter P= 5'd15;
				parameter q= 5'd16;
				parameter r= 5'd17;
				parameter S= 5'd18;
				parameter t= 5'd19;
				parameter u= 5'd20;
				parameter v= 5'd21;
				parameter w= 5'd22;
				parameter X= 5'd23;
				parameter Ya= 5'd24;
				parameter zam= 5'd25;
								
			
initial
begin
             final_path1[0]=5'd0;
				 final_path1[1]=5'd0;
				 final_path1[2]=5'd0;
				 final_path1[3]=5'd0;
				 final_path1[4]=5'd0;
				 final_path1[5]=5'd0;
				 final_path1[6]=5'd0;
				 final_path1[7]=5'd0;
				 final_path1[8]=5'd0;
				 final_path1[9]=5'd0;
	          
            final_path=50'b0;
				
				for(s=0;s<27;s=s+1)
				begin
					for(p=0;p<27;p=p+1)
					dist[s][p]=5'd30;
             end
				 
             dist[0][1]=5'd3;
             dist[1][2]=5'd3;dist[1][13]=5'd3;
             dist[2][3]=5'd1;dist[2][5]=5'd3;dist[2][1]=5'd3;
             dist[3][2]=5'd1;
             dist[4][6]=5'd3;
             dist[5][2]=5'd3;dist[5][6]=5'd1;dist[5][9]=5'd2;
             dist[6][4]=5'd3;dist[6][5]=5'd1;dist[6][16]=5'd3;
             dist[7][12]=5'd2;
             dist[8][9]=5'd1;
             dist[9][5]=5'd2;dist[9][8]=5'd1;dist[9][15]=5'd1;
             dist[10][16]=5'd2;
             dist[11][12]=5'd3;
             dist[12][7]=5'd2;dist[12][13]=5'd1;dist[12][17]=5'd3;dist[12][11]=5'd3;
             dist[13][1]=5'd3;dist[13][12]=5'd1;dist[13][18]=5'd2;
             dist[14][15]=5'd1;
             dist[15][14]=5'd1;dist[15][22]=5'd1;
             dist[16][6]=5'd3;dist[16][10]=5'd2;dist[16][23]=5'd2;
             dist[17][12]=5'd3;
             dist[18][13]=5'd2;dist[18][19]=5'd1;dist[18][20]=5'd1;
             dist[19][18]=5'd1;
             dist[20][18]=5'd1;dist[20][21]=5'd1;dist[20][22]=5'd2;
             dist[21][20]=5'd1;
             dist[22][15]=5'd1;dist[22][20]=5'd2;dist[22][23]=5'd1;dist[22][25]=5'd3;
             dist[23][16]=5'd2;dist[23][22]=5'd1;dist[23][24]=5'd2;
             dist[24][23]=5'd2;
             dist[25][22]=5'd3;

             node[0]=5'd0;
             node[1]=5'd1;
             node[2]=5'd2;
             node[3]=5'd3;
             node[4]=5'd4;
             node[5]=5'd5;
             node[6]=5'd6;
             node[7]=5'd7;
             node[8]=5'd8;
             node[9]=5'd9;
             node[10]=5'd10;
             node[11]=5'd11;
             node[12]=5'd12;
             node[13]=5'd13;
             node[14]=5'd14;
             node[15]=5'd15;
             node[16]=5'd16;
             node[17]=5'd17;
             node[18]=5'd18;
             node[19]=5'd19;
             node[20]=5'd20;
             node[21]=5'd21;
             node[22]=5'd22;
             node[23]=5'd23;
             node[24]=5'd24;
             node[25]=5'd25;
             node[26]=5'd27;
				 deadend[0]= a;
				deadend[1]=d;
				deadend[2]=e;
				deadend[3]=h;
				deadend[4]=iam;
				deadend[5]=K;
				deadend[6]=L;
				deadend[7]=o;
				deadend[8]=r;
				deadend[9]=t;
				deadend[10]=v;
				deadend[11]=Ya;
				deadend[12]=zam;
		   			
   
			
end			
				
				
always@(y)
begin
case(y)

initialise:begin
           done=0;
			
			  
			  for(j=0;j<27;j=j+1)
           begin
            visited[j]=1'b0;
	        end
			  
			  for(j=0;j<27;j=j+1)
           begin 
           strDist[j]=5'd30;
	        end
			  
			  
            for(i=0;i<27;i=i+1)
            begin
             if(node[i]==s_node)
	           begin
	           strDist[i]=0;
		        nextNode=i;
		        visited[nextNode]=1'b1;
		        prevNode[i]=node[26];
		        end
		       else if(node[i]==e_node)
		        begin
		        eNode=i;
		        end
            end
		  
			  
			  prevNode[26]=node[26];
			  
			  Y=distanceUpdate;
			  end
			  
			  
			  
distanceUpdate: begin              
                for(i=0;i<27;i=i+1)
                begin  
                 if(dist[nextNode][i]+strDist[nextNode]<=strDist[i] && visited[i]!=1)
	              begin
	              strDist[i]=dist[nextNode][i]+strDist[nextNode];
	              prevNode[i]=node[nextNode];
	              end
               end
					Y=nextNodeUpdate;
					end
					

nextNodeUpdate:begin
               min=5'd30;
	            visited[nextNode]=1'b1;
					
               for(i=0;i<27;i=i+1)
               begin
	               if(strDist[i]<min && visited[i]!=1)
		            begin
		            min=strDist[i];
		            nextNode=i;
		            end
	            end
					
					if(visited[eNode]==1)
				   Y=finalise;
					else
					Y=distanceUpdate;
					end
					
finalise:	  begin
               final_path1[0]=node[eNode];
					intel=e_node;
				  
	           for (i=1; i<10 ;i=i+1)
	             begin
			       final_path1[i]= prevNode[intel]; // flag is a array which stores the shortest path nodes of final end node
			       intel=prevNode[intel];
				     end
					  done=1;
					 Y=over;
				  end
					  
					 

over:        begin
				 if(!start)
				 Y=over;
				 else 
				 done=0;
				 end 
				 

		      		

endcase
end

always@(posedge clk)
begin
y<=Y;
if(start)
begin
y<=initialise;
end
end


always@(posedge done)
begin
final_path={final_path1[9],final_path1[8],final_path1[7],final_path1[6],final_path1[5],final_path1[4],final_path1[3],final_path1[2],final_path1[1],final_path1[0]};
end




 

////////////////////////YOUR CODE ENDS HERE//////////////////////////
endmodule
/////////////