%% parametry poczatkowe
time = 100;
ep = 0.005;
y_zad = 0;
P = 0;
I = 0;
D = 0;

%% model
nazwa = 'G1';
G_temp = G1;
name_tabG1 = [0.02 0.05 0.1 0.2 0.3 0.5 0.7 1 1.2 1.5 2 4 6 8 10 20 50 100 200 500 1000];
name_tabG2 = [0.01 0.02 0.05 0.1 0.2 0.3 0.5 0.7 1 1.2 1.5 2 4 6 8 10 20 50 100 200 500];
name_tabG3 = [0.005 0.01 0.02 0.05 0.1 0.2 0.5 2 5 10];
name_tabG4 = [3 4 5 6 7 8];
name_tabG5 = [1 2 3 4 5 6 7 8 9]/10;
name_tabG6 = [0.01 0.02 0.05 0.1 0.2 0.3 0.5 0.7 1]; %L+T=1
name_tabG7a = [0.01 0.02 0.05 0.1 0.2 0.3 0.5 0.7 1]; %L+T1=1
name_tabG7b = [1 2 5 10]; %T
name_tabG8 = [1 2 3 4 5 6 7 8 9 10 11]/10;
name_tabG9 = [1 2 3 4 5 6 7 8 9 10]/10;

name_tab = name_tabG1;

for i = 1:size(G_temp, 1)
    for j = 1:size(G_temp, 2)
        
        G = G_temp(i,j);
        y_zad = 0;
        I = 0;
        D = 0;
        
        %% szukanie parametrów PID
        sim('model');    
        
        resp = system_response.data(:,1);
        b = max(resp) - min(resp);
        a = 2;
        k_kr = 4*b/(pi()*a);

        my_ind = find(diff(relay_response.data(:,1))==2);
        T_osc = (my_ind(end) - my_ind(end-1))/1000;

        plot_num_ind = 1;
        plot_num = 3;
        if b/2 < 1.2*ep
            plot_num = 4;
        end
        
        plot_num = 4;
        
        
        Rr = pi()/(4)*sqrt((b/2)^2 - ep^2);
        Ir = (pi()*ep)/(4);
        Mr = sqrt(Rr^2 + Ir^2);
        fi = pi() -atan(Ir/Rr);
        w_osc = 2*pi()/T_osc;
        
        subplot(plot_num,1,plot_num_ind);
        plot_num_ind = plot_num_ind + 1;
        plot(relay_response.time, relay_response.data(:,1), ...
        system_response.time, system_response.data(:,1));

        grid on;
        title('Sygna³ steruj¹cy oraz odpowiedŸ obiektu.');
        xlabel('czas');
        axis([0 time -1.5 1.5]);
        
        y_zad = 1;

        % P
%         P = k_kr/2;
%         sim('modelPID');
%         subplot(plot_num,1,plot_num_ind);
%  %       plot_num_ind = plot_num_ind + 1;
%         plot(pid_response.time, pid_response.data(:,1), ...
%              response.time, response.data(:,1));
% 
%         ax = axis;
%         if ax(4)==1
%             axis([0 time 0 1.3]);
%         end
% 
%         grid on;
%         title('Refulator P');

        %xlabel('czas');
      %  if b/2 < 1.2*ep
         if 1==1
            % PI
            P = k_kr/2.2;
            I = T_osc/1.2;
            sim('modelPID');
            subplot(plot_num,1,plot_num_ind);
            plot_num_ind = plot_num_ind + 1;
            plot(response.time, response.data(:,1), ...
                 pid_response.time, pid_response.data(:,1));

            ax = axis;
            if ax(4)==1
                axis([0 time 0 1.3]);
            end

            grid on;
            title('Regulator PI - nastawy ZN');
            %xlabel('czas');
        end
        % PID
        
        P = k_kr/1.7;
        I = T_osc/2;
        D = T_osc/8;

        sim('modelPID'); 
        subplot(plot_num,1,plot_num_ind);
        plot_num_ind = plot_num_ind + 1;
        plot(pid_response.time, pid_response.data(:,1),'r', ...
             response.time, response.data(:,1),'b');
        
        
        ax = axis;
        if ax(4)==1
            axis([0 time 0 1.3]);
        end 
        grid on;
        title('Regulator PID - nastawy ZN');
        

        I = 2/w_osc*tan(1/2*(fi-pi()/4));
        P = 2/Mr*w_osc*I/((w_osc*I)^2 + 4);
        D = I/4;
        sim('modelPID');
        subplot(plot_num,1,plot_num_ind);
        plot(pid_response.time, pid_response.data(:,1),'r', ...
             response.time, response.data(:,1),'b');
         
         
        ax = axis;
        if ax(4)==1
            axis([0 time 0 1.3]);
        end 
        
        
        grid on;
        title('Regulator PID - nastawy AH');
        xlabel({' ';strjoin({'Obiekt ',nazwa,', T = ', num2str(name_tab(j))},'')},'FontSize',12);
       % xlabel({' ';strjoin({'Obiekt ',nazwa,', T = ', num2str(name_tabG7b(i)),', L = ', num2str(name_tab(j)),', T1 = ',num2str(1 - name_tab(j))},'')},'FontSize',12);

        saveas(gcf, strjoin({nazwa,'\',nazwa,'_',int2str(i),'_',int2str(j),'.png'},''),'png');
        close;
    end
end
%% koniec generowania wykresów

%% 