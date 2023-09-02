function [Inputs,FLAG_terminate ] = ROB535_ControlProject_part2_Team12(TestTrack, Xobs,current_state)

    addpoint = [1473 1474 1475;821 824 827];
    TestTrack.cline = [TestTrack.cline addpoint];
    TestTrack.theta = [TestTrack.theta TestTrack.theta(end) TestTrack.theta(end) TestTrack.theta(end)];
    addpoint2 = [1462 1463 1464;822.6 824.4 826.2];
    TestTrack.bl = [TestTrack.bl addpoint2];
    addpoint3 = [1484 1485 1486;819.4 824.1 828.8];
    TestTrack.br = [TestTrack.br addpoint3];
    end_goal = TestTrack.cline(:, end);

x0=current_state;
    xinit = x0;
    s_pos = [x0(1); x0(3)];
    s_theta = x0(5);
    s_index = 1;
    n_step = 3;
    track_lane = -1; 
    if((s_theta>=pi/2)&&(s_theta<=3*pi/2))
        left_mark = -1;
    else
        left_mark = 1;
    end
    
    if((s_theta>=0)&&(s_theta<=pi))
        up_mark = 1;
    else
        up_mark = -1;
    end
    
    Cobs = generate_cobs(Xobs);
    Kf = 15.0;
    Kl = 0.6;
    dt = 0.01;
    Fb = 150;

    iteration_time = 1;

    Inputs = [];
    curritr=1;
    while  curritr
        [ob_state, ~, obs_lane] = detect_obs(s_pos, s_theta, Cobs, TestTrack, Xobs, up_mark);

        if(ob_state)
            track_lane = 1-obs_lane;
        else
            track_lane = -1;
        end
      
        [t_pos, ~, ~, ~, ~, ~] = get_target(s_pos, TestTrack, track_lane);
        [l_error, f_error] = get_loss(s_pos, t_pos, s_theta, left_mark, up_mark);     
        fx = Kf*(1.2*f_error-x0(2)) + Fb;  
        deltaf = 1.2*l_error*Kl;
        input = repmat([deltaf, fx], n_step, 1);
        Inputs = [Inputs; repmat([deltaf, fx], n_step-1, 1)];

        [Y,~] = forwardIntegrateControlInput(input,x0);
      
        x0 = Y(end, :);
       
        if((s_theta>=pi/2)&&(s_theta<=3*pi/2))
            left_mark = -1;
        else
            left_mark = 1;
        end
        if((s_theta>=0)&&(s_theta<=pi))
            up_mark = 1;
        else
            up_mark = -1;
        end
   
        iteration_time = iteration_time + 1;
        if iteration_time==100
            [sY, ~] = forwardIntegrateControlInput(Inputs,xinit);
            x0 = sY(end, :); 
            curritr=0;
        end
          
        s_theta = x0(5);
        s_pos = [x0(1); x0(3)];
        
       if(norm([current_state(1);current_state(3)]-end_goal)<10)
           FLAG_terminate = 1;
      
       else
            FLAG_terminate = 0;
       end
    end
    
end


function Cobs = generate_cobs(Xobs)
    obs_num = size(Xobs, 2);
    Cobs = zeros(obs_num, 2);
    for i=1:1:obs_num
        obs_pos = Xobs{1, i};
        Cobs(i, 1) = mean(obs_pos(:, 1));
        Cobs(i, 2) = mean(obs_pos(:, 2));
    end
end
function [l_error, f_error] = get_loss(c_pos, t_pos, c_theta, left_mark, up_mark)
    
    l_threshold = 5;
    f_threshold = 0;
    k = tan(c_theta);
    
    l_error = ((t_pos(2) - c_pos(2)) - k*(t_pos(1) - c_pos(1)))/sqrt(1+k^2);
    l_error = left_mark*l_error;
    if(abs(l_error)<l_threshold)
        l_error = l_error/l_threshold;
    else
        l_error = sign(l_error)*1;
    end
    k = -1/k;
    f_error = ((t_pos(2) - c_pos(2)) - k*(t_pos(1) - c_pos(1)))/sqrt(1+k^2);
    f_error = up_mark*f_error;
    if(abs(f_error)<f_threshold)
        f_error = 0;
    else
        f_error = sign(f_error)*(abs(f_error)-f_threshold);
    end
end

function [ob_state, obs, track_lane] = detect_obs(s_pos, s_theta, Cobs, TestTrack, Xobs, up_mark)
    obs_threshold = 50;
    obs_num = size(Cobs,1);
    n_index = 0;
    min_d = Inf;
    min_d2 = Inf;
    track_lane = [];
    for i=1:1:obs_num
        d = norm(s_pos - Cobs(i, :)');
        if(d<min_d)
            min_d2 = min_d;
            n2_index = n_index;
            min_d = d;
            n_index = i;
        end
    end
    if(min_d<obs_threshold)
        obs = Cobs(n_index, :);
        [~, ~, l_pos, r_pos, ~, ~] = get_target(obs', TestTrack, 0);
        if(obs_pass(s_pos, s_theta, Xobs{1, n_index}, up_mark))
            if((min_d2<obs_threshold)&&~(obs_pass(s_pos, s_theta, Xobs{1, n2_index}, up_mark)))
              obs = Cobs(n2_index, :);
                obs2l = norm(obs'-l_pos);
                obs2r = norm(obs'-r_pos);
                if(obs2l > obs2r)
                    track_lane = 1; 
                else
                    track_lane = 0; 
                end
                ob_state = true;
            else
                ob_state = false;
                obs = []; 
            end
        else

            obs2l = norm(obs'-l_pos);
            obs2r = norm(obs'-r_pos);
            if(obs2l > obs2r)
                track_lane = 1; 
            else
                track_lane = 0; 
            end
            ob_state = true;
        end 
    else
        ob_state = false;
        obs = [];
    end
end

function pass_state = obs_pass(s_pos, s_theta, Obs, up_mark)
    pass_state = true;
    for i=1:1:4
        obs_pos = Obs(i, :);
        [~, f_error] = get_loss(s_pos, obs_pos, s_theta, -1, up_mark);
        if f_error > 0
            pass_state = false;
        end
    end
end
function [t_pos, t_theta, l_pos, r_pos, b_theta, s_index] = get_target(c_pos, track_data, track_lane)
    
    cpoints = track_data.cline;
    min_d = Inf;
    close_i = 0;
    for i=1:1:size(cpoints, 2)
        d = norm(cpoints(:, i)-c_pos);
        if d<min_d
            min_d = d;
            close_i = i;
        end
    end
    if close_i ~= size(cpoints, 2)
        if track_lane == 0
            t_pos = (cpoints(:, close_i+1)+track_data.bl(:, close_i+1))/2;
        elseif track_lane == 1
            t_pos = (cpoints(:, close_i+1)+track_data.br(:, close_i+1))/2;
        else
            t_pos = cpoints(:, close_i+1);
        end
        t_theta = track_data.theta(close_i+1);
        
        l_pos = track_data.bl(:, close_i);
        r_pos = track_data.br(:, close_i);
        b_theta = track_data.theta(close_i);
        s_index = close_i;
    else
        if track_lane == 0
            t_pos = (cpoints(:, close_i)+track_data.bl(:, close_i))/2;
        elseif track_lane == 1
            t_pos = (cpoints(:, close_i)+track_data.br(:, close_i))/2;
        else
            t_pos = cpoints(:, close_i);
        end
        
        t_theta = track_data.theta(close_i);
        l_pos = track_data.bl(:, close_i);
        r_pos = track_data.br(:, close_i);
        b_theta = track_data.theta(close_i);
        s_index = close_i;
    end
end