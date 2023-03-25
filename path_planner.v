// SM : Task 2 C : Path Planning
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.
This file is used to design path planner.

Recommended Quartus Version : 19.1
The submitted project file must be 19.1 compatible as the evaluation will be done on Quartus Prime Lite 19.1.

Warning: The error due to compatibility will not be entertained.
-------------------
*/

//Path Planner design
//Parameters : node_count : for total no. of nodes + 1 (consider an imaginary node, refer discuss forum),
//					max_edges : no. of max edges for every node.


//Inputs  	 : clk : 50 MHz clock, 
//				   start : start signal to initiate the path planning,
//				   s_node : start node,
//				   e_node : destinat|path_planner
//
//Output     : done : Path planning completed signal,
//             final_path : the final path from start to end node.



//////////////////DO NOT MAKE ANY CHANGES IN MODULE//////////////////
module path_planner
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
reg [7:0] distance[0:25];
reg [4:0] predecessor[0:25];

// Declare internal variables
reg [7:0] d[0:25];
reg [4:0] p[0:25];
reg [4:0] u, v;
reg [4:0]final_path1[0:9];

// Initialize the distance and predecessor arrays
initial
begin
for(u=0;u<26; u=u+1)
begin
distance[u] <= 8'hFF;
predecessor[u] <= 5'h1F;
end
end


// Implement the Dijkstra algorithm
always @(posedge clk) 
begin
	if (start) 
	begin
		distance[s_node] <= 8'h00;
		p[s_node] <= s_node;
		done=0;
		for (v = 0; v < 26; v = v + 1) 
			begin
				if (v != s_node) 
				begin
					d[v] <= distance[v];
					p[v] <= predecessor[v];
				end
			end
		for (u = 0; u < 26; u = u + 1) 
		begin
			for (v = 0; v < 26; v = v + 1) 
			begin
				if (adjacent(u, v)) 
				begin
					if (d[v] > (d[u] + weight(u, v))) 
					begin
						d[v] <= (d[u] + weight(u, v));
						p[v] <= u;
					end
				end
			end
			for (u = 0; u < 26; u = u + 1)
			 begin
			  distance[u] <= d[u];
			  predecessor[u] <= p[u];
			 end
		end
		v=predecessor[e_node];
		final_path1[0]=e_node;
      for(u=1;u<10;u=u+1)
        begin
         final_path1[u]=v;
         v=predecessor[v];
        end
      final_path={final_path1[9],final_path1[8],final_path1[7],final_path1[6],final_path1[5],final_path1[4],final_path1[3],final_path1[2],final_path1[1],final_path1[0]};
		done=1;
	end
end


// Adjacent function returns 1 if there is an edge between
// nodes u and v, 0 otherwise
function [1:0]adjacent;
input [4:0] u, v;
begin
reg [1:0]graph[0:25][0:25]={ 0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                        2'd1,0,2'd1,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,
								0,2'd1,0,2'd1,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,2'd1,0,0,0,2'd1,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,2'd1,2'd1,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,2'd1,0,0,2'd1,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,2'd1,0,0,0,2'd1,0,2'd1,0,0,0,2'd1,0,0,0,0,0,0,0,0,
								0,2'd1,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,2'd1,0,0,0,0,0,0,0,2'd1,0,0,0,
								0,0,0,0,0,0,2'd1,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,2'd1,2'd1,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,2'd1,2'd1,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,2'd1,0,0,2'd1,0,2'd1,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0,0,0,2'd1,0,2'd1,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2'd1,0,0,0};
								
					adjacent=graph[u][v];

end
endfunction

// Weight function returns the weight of the edge between
// nodes u and v
function [7:0]weight;
input [4:0] u, v;
begin
reg [7:0]graphw[0:25][0:25]={0,8'd3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                        8'd3,0,8'd3,0,0,0,0,0,0,0,0,0,0,8'd3,0,0,0,0,0,0,0,0,0,0,0,0,
								0,8'd3,0,8'd1,0,8'd3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,8'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,8'd3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,8'd3,0,0,0,8'd1,0,0,8'd2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,8'd3,8'd1,0,0,0,0,0,0,0,0,0,0,8'd3,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,8'd2,0,0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,8'd1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,8'd2,0,0,8'd1,0,0,0,0,0,0,8'd1,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd2,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,8'd3,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,8'd2,0,0,0,8'd3,0,8'd1,0,0,0,8'd3,0,0,0,0,0,0,0,0,
								0,8'd3,0,0,0,0,0,0,0,0,0,0,8'd1,0,0,0,0,0,8'd2,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd1,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,8'd1,0,0,0,0,8'd1,0,0,0,0,0,0,0,8'd1,0,0,0,
								0,0,0,0,0,0,8'd3,0,0,0,8'd2,0,0,0,0,0,0,0,0,0,0,0,0,8'd2,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,8'd3,0,0,0,0,0,0,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,8'd2,0,0,0,0,0,8'd1,8'd1,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd1,0,0,0,0,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd1,0,0,8'd1,8'd2,0,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd1,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd1,0,0,0,0,8'd2,0,0,8'd1,0,8'd3,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd2,0,0,0,0,0,8'd1,0,8'd2,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd2,0,0,
								0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8'd3,0,0,0};
								weight=graphw[u][v];
end
endfunction

endmodule
		
		/*for (n=1; n<10 ;n=n+1)
	   begin : ABC
			final_path1[n]= predecessor[flag]; // flag is a array which stores the shortest path nodes of final end node
			flag = predecessor[flag];
			if ( flag == s_node ) 
			begin
			done=1;
			disable ABC;
			end
			
		end
	  
    end
  end
end

always@(posedge done)
begin
final_path={final_path1[9],final_path1[8],final_path1[7],final_path1[6],final_path1[5],final_path1[4],final_path1[3],final_path1[2],final_path1[1],final_path1[0]};
end

endmodule*/