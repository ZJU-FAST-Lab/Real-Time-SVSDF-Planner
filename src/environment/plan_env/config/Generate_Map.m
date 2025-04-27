clear

file_head = "obs";
file_tail = ".yaml";


start_x = -9.5;
start_y = -9.5;

x_min = -10.0;
x_max = 10.0;
y_min = -10.0;
y_max = 10.0;

box_size = 0.5;
min_box_interval = 1.0;
yaw = 0.0;
obs_num = 50000;

data = []; % center: x,y  size: x,y   yaw
yamldata.map_box = [];
yamldata.map_size = [x_min, x_max, y_min, y_max];

error_num = 0;
error_max = 1000;

for i = 1:10
    file_name = file_head + num2str(i) + file_tail;
    if exist(file_name, 'file') == 2
        continue;
    end
    
    obs_i = 1;
    while obs_i <= obs_num && error_num < error_max
        coor_x = (x_max - x_min) * rand() + x_min;
        coor_y = (y_max - y_min) * rand() + y_min;

        satisfy = true;
        for data_i = 1:size(data,1)
            if abs(coor_x - data(data_i, 1)) < min_box_interval && abs(coor_y - data(data_i, 2)) < min_box_interval
                satisfy = false;
                break;
            end
        end
        if abs(coor_x - start_x) < box_size*1.0 && abs(coor_y - start_y) < box_size*1.0
            satisfy = false;
        end

        if satisfy
            data(end+1, :) = [coor_x, coor_y, box_size, box_size, yaw];
            obs_i = obs_i + 1;
        else
            error_num = error_num + 1;
        end
    end

    for data_i = 1:size(data, 1)
        yamldata.map_box = [yamldata.map_box, data(data_i, :)];
    end

    s = yaml.dump(yamldata);
    yaml.dumpFile(file_name, yamldata);
    break;

end

if error_num == error_max
    disp("Error generating obstacles, possibly due to too many obstacles or too large size!!!")
end
