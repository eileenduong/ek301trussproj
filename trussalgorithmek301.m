

% ------------------ input ------------------------------- 

% Connection matrix for joints (J rows x M columns) 
C = [1 1 0 0 0 0 0; 
     1 0 1 0 1 0 1; 
     0 1 1 1 0 0 0; 
     0 0 0 1 1 1 0
     0 0 0 0 0 1 1]; 
 
 % Connection matrices for support forces along each axis (J rows x 3 Columns) 
 % [Sx1, Sy1, Sy2] 
 Sx = [1 0 0; 
       0 0 0; 
       0 0 0; 
       0 0 0
       0 0 0];
   
 Sy = [0 1 0; 
       0 0 0;
       0 0 0; 
       0 0 0
       0 0 1]; 
   
% Location vectors of joints    
X = [ 0 1 0.5 1.5 2.1]; 
Y = [ 0 0   1 1.3   0]; 

% Load vector [x x x x y y y y] 
L = [ 0 0 0 0 0 0 1 0 0 0]; 

%---------------------------------------------------------

% algorithm 

% preallocate a vector 
% double number of rows and add 3 columns and create matrix A
[C_rows, C_columns] = size(C); 
A = zeros(C_rows *2, C_columns + 3);  

% return size of newly made A matrix 
[A_rows, A_columns] = size(A);  

% fill in the A matrix with 1's and 0's from the C, Sx, and Sy matrices
%from C matrix (columns stay the same) 
A(1:C_rows, 1: C_columns) = C; 
A(C_rows+1:A_rows, 1:C_columns) = C; 
%from Sx matrix (the columns stay the same)
A(1:C_rows, A_columns-2:A_columns) = Sx; 
%from Sy matrix (the columns stay the same)
A(C_rows+1:A_rows, A_columns-2:A_columns) = Sy; 


% declare global variable
global first; 
running_length = 0; 


for i = 1:C_rows     
    
    members_touched_by_jointI  = find(C(i,:)); 
    % this should result in a matrix that has elements 
    % means that joint i touches members _element value_ 
    
    %return length of the vector  
    length_of_index = length(members_touched_by_jointI); 
       
    %traverse through columns 
    for j = 1:length_of_index
        other_joints_touched_by_members = find(C(:,members_touched_by_jointI(j))); 
               
        % transpose into horizontal vector 
        other_joints_touched_by_members = other_joints_touched_by_members'; 
        
       
         for k = 1:2
             if other_joints_touched_by_members(k) ~= i
                 first = other_joints_touched_by_members(k); 
             end 
         end
         
         %top half of A matrix (x)
         A(i,members_touched_by_jointI(j)) =  (X(first)-X(i))/sqrt((X(first)-X(i))^2 +(Y(first)-Y(i))^2);
         %bottom half of A matrix (y)
         A(i+C_rows,members_touched_by_jointI(j)) =  (Y(first)-Y(i))/sqrt((X(first)-X(i))^2 +(Y(first)-Y(i))^2);
    end 
end

% calculate load 
load = sum(L); 

% calculate T vector 
T = A\-L'; 
lengthofT = length(T); 

% calculate running distance
for j = 1:C_columns
        vec = find(C(:,j))';
        vec1 = (vec(1));
        vec2 = (vec(2));
        distance = sqrt((X(vec1)-X(vec2))^2 + (Y(vec1)-Y(vec2))^2);
        running_length = running_length + distance; 
end

% calculate totalcost 10J+1L (finish later) 
totalcost = 10*(C_rows) + running_length;    

% calculate ratio cost/load ratio 
ratio = load/totalcost; 


% --------------------- output ----------------------------
% reproduce input file header and print the file execution date  
fprintf("\n")


% print applied load 
fprintf("Load: %d N \n", load) 

fprintf("Member forces in Newtons \n")    

for i = 1:lengthofT-3
    % print label of member and magnitude 
    fprintf("m%d: %.3f ", i, abs(T(i)))
    
    % print tension or compression
    if T(i) ==0 
        fprintf("0 \n"); 
    elseif T(i) > 0 
            fprintf("(T) \n"); 
        else 
            fprintf("(C) \n"); 
    end 
end 

% print reaction forces 
fprintf("Reaction forces in Newtons: \n")    
fprintf("Sx1: %.2f \n", T(lengthofT-2))
fprintf("Sy1: %.2f \n", T(lengthofT-1))
fprintf("Sy2: %.2f \n", T(lengthofT))

% print costs 
fprintf("Cost of truss: $%.2f \n", totalcost)
fprintf("Theoretical max load/cost ratio in N/$: %.4f \n", ratio)










 
   
