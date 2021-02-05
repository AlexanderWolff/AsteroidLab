

%t = tcpclient('192.168.0.11',30002);
%write(t, unicode2native(output,'US-ASCII'));

while t.BytesAvailable > 0
    
   read(t,t.BytesAvailable); 
    
end