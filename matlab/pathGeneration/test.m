 m = 1000000; % ... number of rows
 n = 20; % ... dimension of vector
 M = randi([1 n],m,n);
 
  testloc = 123456;
  testloc=testloc:testloc+5;
 x = M(testloc,:);
 
 tic;
 [~,loc1] = ismember(x,M,'rows'); 
 toc;
 %Elapsed time is 4.058689 seconds.
 
  tic;
  loc= find(any(all(bsxfun(@eq,reshape(x.',1,n,[]),M),2),3)); 
 toc;