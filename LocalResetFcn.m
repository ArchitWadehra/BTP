function in = LocalResetFcn(in)

% Randomize initial voltage
h = 3*randn + 10;
while h <= 0 || h >= 20
h = 3*randn + 10;
end
blk = 'rlwatertank/Water-Tank System/H';
in = setBlockParameter(in,blk,'InitialCondition',num2str(h));

end