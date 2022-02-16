% dubinsParameters
%   - Find Dubin's parameters between two configurations
%
% Modified:  
%   - 4/13/2010 - RWB
%
% input is:
%   start_node  - [wn_s, we_s, wd_s, chi_s, 0, 0]
%   end_node    - [wn_e, wn_e, wd_e, chi_e, 0, 0]
%   R           - minimum turn radius
%
% output is:
%   dubinspath  - a matlab structure with the following fields
%       dubinspath.ps   - the start position in re^3
%       dubinspath.chis - the start course angle
%       dubinspath.pe   - the end position in re^3
%       dubinspath.chie - the end course angle
%       dubinspath.R    - turn radius
%       dubinspath.L    - length of the Dubins path
%       dubinspath.cs   - center of the start circle
%       dubinspath.lams - direction of the start circle
%       dubinspath.ce   - center of the end circle
%       dubinspath.lame - direction of the end circle
%       dubinspath.w1   - vector in re^3 defining half plane H1
%       dubinspath.q1   - unit vector in re^3 along straight line path
%       dubinspath.w2   - vector in re^3 defining position of half plane H2
%       dubinspath.w3   - vector in re^3 defining position of half plane H3
%       dubinspath.q3   - unit vector defining direction of half plane H3
% 

function dubinspath = dubinsParameters(start_node, end_node, R)

  ell = norm(start_node(1:2)-end_node(1:2));
  if ell<2*R
      disp('The distance between nodes must be larger than 2R.');
      dubinspath = [];
  else
    
    ps   = start_node(1:3);
    chis = start_node(4);
    pe   = end_node(1:3);
    chie = end_node(4);
    

    crs = 
    cls = 
    cre = 
    cle = 
    
   
    % compute L1

    L1 = 
    % compute L2
    L2 = 
    % compute L3
    L3 = 
    % compute L4
    L4 = 
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    switch(idx)
        case 1
        case 2   
        case 3
        case 4
    end
    
    % assign path variables
    dubinspath.ps   = 
    dubinspath.chis = 
    dubinspath.pe   = 
    dubinspath.chie = 
    dubinspath.R    = 
    dubinspath.L    = 
    dubinspath.cs   = 
    dubinspath.lams = 
    dubinspath.ce   = 
    dubinspath.lame = 
    dubinspath.w1   = 
    dubinspath.q1   = 
    dubinspath.w2   = 
    dubinspath.w3   = 
    dubinspath.q3   = 
  end
end


