function func_newObj(ClientHandle, Color)
writeTCP(ClientHandle,sprintf("NewObj:%d",Color));
pause(0.054);