clc, clear variables
%%

% 2 khz
max_num_of_floats = 2000000 / (4 * (8 + 2) * 2000)


% openlager
file_id = fopen('LOG000.TXT');


num_of_floats = fread(file_id, 1, 'uint8')

data_raw = fread(file_id, 'single');
length(data_raw)

fclose(file_id);


%%

data.values = reshape(data_raw, [num_of_floats, length(data_raw)/num_of_floats]).';
            
data.time = cumsum(data.values(:,1)) * 1e-6;
data.time = data.time - data.time(1);

data.values = data.values(:,2:end);


%%

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean dTime = %0.2f musec, ', ...
                'Std. dTime = %0.2f musec, ', ...
                'Median dTime = %0.2f musec'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (musec)')
ylim([0 1.2*max(diff(data.time * 1e6))])
xlim([0 data.time(end-1)])

figure(2)
plot(data.time, data.values), grid on
xlabel('Time (sec)')
xlim([0 data.time(end)])

