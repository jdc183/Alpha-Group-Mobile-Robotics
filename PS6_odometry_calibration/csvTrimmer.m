for i = 1:15
    filename = ['circle', (num2str(i)), '.csv'];
    data = csvread(filename);
    n = size(data,1);
    for j=n:-1:2
        if (data(j,1)==data(j-1,1) && data(j,2)==data(j-1,2))
            data(j,:)=[];
        end
    end
    newFilename = ['trimd' filename];
    dlmwrite(newFilename,data,'precision',12);
end