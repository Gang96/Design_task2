%set up number of samples for per period.
samples = 10000;
freqz = 50.0;
i = 1:1:samples;
offset = 2;
A = 2;
a = A*sin(2*pi*freqz*(i-1)/samples);
%give offset to the sampled data, make sure input data starts from zero, which is measurable for Mega2560.
a = a + offset;
%set up channels for DAQ
ao = analogoutput('nidaq','Dev3');
addchannel(ao,0);
%set up time information, push out samples
while(1) %Output the signal forever
for n = 1:length(a)
putsample(ao,a(n));
end
end