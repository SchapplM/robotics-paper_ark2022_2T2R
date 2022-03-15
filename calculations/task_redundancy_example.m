% Example for functional redundancy with 2T2R and 2T3R EE DoF
% This creates Fig. 2 and 3 of the paper

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

close all
clear
clc


%% User Settings
usr_create_anim = false;
usr_plot_robot = true; % to enable robot plot (Fig. 2)
usr_plot_convergence = true;  % to enable convergence plot (Fig. 3)
usr_plot_debug = false; % Debug-Plots showing the robot and all frames
usr_recreate_mex = false;
use_mex_functions = true;
usr_ptno = 2; % number of the point to select from the predefined ones

% Prüfung repräsentativer Roboter
Robots = {
         {'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1'}, ...
         {'S6RRRRRR10V2', 'S6RRRRRR10V4_UR5'}, ...    
         };

if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

% Einstellungen
this_dir = fileparts(which('task_redundancy_example'));
respath = fullfile(this_dir);
paperfig_path = fullfile(respath, '..', 'paper', 'figures');
mkdirs(respath);

% Endeffektor-Transformation ungleich Null festlegen, um zu prüfen, ob die
% Implementierung davon richtig ist
r_W_E = [-0.05;-0.07;-0.09];  % EE-Transf. vorne weg zeigend
phi_W_E = [5; 10+180; 20]*pi/180;
% formatting of the lines in the plot
plot_lw = 1.0;
plot_ms = 6; % default
lineformat = {[0 255 255]/255,  '', '-', 12, plot_lw, plot_ms; ... % cyan
          [0 80 155 ]/255, '^', '-', 4, plot_lw, plot_ms; ... %imesblau
          [231 123 41 ]/255, 'o', '-', 5, plot_lw, plot_ms; ... %imesorange
          'r', 'x', '-', 7, plot_lw, plot_ms; ...
          'k', 's',  '-', 9, plot_lw, plot_ms; ...
          'g', 'v', '-', 34, plot_lw, plot_ms};
%% Alle Robotermodelle durchgehen
Robot_Data = Robots;
use_RobotNr = 1;
for Robot_Data_i = use_RobotNr:use_RobotNr
  SName = Robot_Data{Robot_Data_i}{1};
  RName = Robot_Data{Robot_Data_i}{2};
   
  %% Klasse für seriellen Roboter erstellen
  % Instanz der Roboterklasse erstellen
  if usr_recreate_mex
    serroblib_create_template_functions({SName}, false, false); %#ok<*UNRCH>
    matlabfcn2mex({'S6RRRRRR10V2_invkin_eulangresidual'}, false);
    matlabfcn2mex({'S6RRRRRR10V2_invkin_traj'});
  else
    serroblib_update_template_functions({SName});
  end
  RS = serroblib_create_robot_class(SName, RName);
  RS.fill_fcn_handles(use_mex_functions, true);
  % Grenzen festlegen (für Zusatz-Optimierung)
  RS.qlim = repmat([-pi, pi], RS.NQJ, 1);
  RS.qDlim = repmat([-4*pi, 4*pi], RS.NQJ, 1); % 2rpm
  RS.qDDlim = repmat([-100, 100], RS.NQJ, 1); % entspricht 1.5 rpm in 100ms
  qlim   = cat(1, RS.qlim);
  qDlim  = cat(1, RS.qDlim);
  qDDlim = cat(1, RS.qDDlim);
  RS.update_EE(r_W_E, phi_W_E, []);
  I_EE_3T3R = logical([1 1 1 1 1 1]);
  I_EE_3T2R = logical([1 1 1 1 1 0]);
  I_EE_2T2R = logical([1 1 0 1 1 0]);
  I_EE_2T3R = logical([1 1 0 1 1 1]);
  RS.I_EE = I_EE_3T3R;
  RS.I_EE_Task = I_EE_3T3R; % zu Beginn auf 3T3R
  % hollow cylinder with diameter 50mm (for plot/animation)
  RS.DesPar.seg_par = repmat([5e-3,50e-3],RS.NL,1);
      
  %% Arbeitspunkte definieren
  
  if use_RobotNr == 1 % KUKA
  %   x_mid = [1.2,0,1, 0,pi/2,0]';
    x_mid = [1.2,0,1, 0,pi/2,0]'; % -> EE nach vorn zeigend
%     x_mid = [0.5,0,1, 0,0,0]';  % -> EE nach hinten zeigend
    d = 0.6;
    cell_width = 3;
  elseif use_RobotNr == 2 % UR5
    x_mid = [0.3,0,0.5, 0,0,0]';  % für UR5 muss Arbeitsbereich eingschränkt werden
    d = 0.25;
    cell_width = 3;
  end
  
  k=1; XE = x_mid';
  for i = 1:cell_width
    switch i
      case 1  % links/rechts zum Mittelpunkt
%         k = k+1; XE(k,:) = x_mid' + [ 0,d,0, 0,0,0];
%         k = k+1; XE(k,:) = x_mid' + [ 0,-d,0, 0,0,0];
      case 2  % links/rechts über dem Mittelpunkt
%         k = k+1; XE(k,:) = x_mid' + [ 0,0,d, 0,0,0];
        k = k+1; XE(k,:) = x_mid' + [ 0,d,d, 0,0,0];
        k = k+1; XE(k,:) = x_mid' + [ 0,-d,d, 0,0,0];
      case 3  % links/rechts unter dem Mittelpunkt
%         k = k+1; XE(k,:) = x_mid' + [ 0,0,-d, 0,0,0];
        k = k+1; XE(k,:) = x_mid' + [ 0,d,-d, 0,0,0];
        k = k+1; XE(k,:) = x_mid' + [ 0,-d,-d, 0,0,0];
      otherwise
        error('Case not expected');
    end
  end
  
%   % Manuelle Anpassung -> EE-Tranf. nach vorne zeigend
%   % P1:P5
  XE(1,4:5) = [5, 90+20]*pi/180;
  XE(2,4:5) = [20, 90+45]*pi/180;
  XE(3,4:5) = [20, 90+20]*pi/180;
  XE(4,4:5) = [40, 90+45]*pi/180;
  XE(5,4:5) = [-40, 90+45]*pi/180;
  
  % Adapt the coordinates for the 3T2R problem. Shift position such that
  % the new position is in the mid range of the dz from the 2T2R problem.
  xlim_2T2R = [NaN(2,2); [-0.5 -0.1]; NaN(3,2)];
  XE_dz0 = XE;
  for i = 1:size(XE,1)
    T_i = RS.x2t(XE(i,:)');
    dz0 = mean(xlim_2T2R(3,:));
    delta_0 = T_i(1:3,1:3)*[0;0;dz0];
    XE_dz0(i,1:3) = XE(i,1:3) + delta_0';
  end

  q0 = RS.qref;
                                                                                    
  % Debug-plot aller Punkte
  if usr_plot_debug
  s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
  s_plot_KS = struct( 'ks', [8, RS.NJ+2], 'straight', 0, 'mode', 2);
  change_current_figure(11);clf;hold on;grid on;
  xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
  view(3);
  RS.plot( q0, s_plot );
  title(sprintf('Arbeitspunkte: %s', RS.descr));
  for k = 1:size(XE,1)
    T_0_Ek = RS.x2t(XE(k,:)');
    trplot(T_0_Ek, 'frame', sprintf('D%d',k), 'rgb', 'length', 0.4)
  end
  end
  if ~(any(1:size(XE,1) == usr_ptno))
    error('Punkt ist nicht in XE enthalten -> neu wählen');
  end
  %% Compute IK with different settings
  Names_Methods = {'3T2R', '2T2R', '2T2R_limred', '2T3R_limred'};
  Names_Methods_Leg = {'3T2R', '2T2R', '2T2R*', '2T3R*'};
  Phirt_tol = 1e-6;
  s_default = struct( ...
    'scale_lim', 0, ...
    'wn', zeros(RS.idx_ik_length.wnpos,1), ...
    'maxstep_ns', 1e-6, ...
    'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
    'maxrelstep', 0.05, ...
    'xlim', xlim_2T2R, ...
    'retry_limit', 0);
  Stats_all = cell(length(Names_Methods), size(XE,1));
  n_max = 2500;
  for k = 1:length(Names_Methods)
    XE_k = XE;
    Name_k = Names_Methods{k};
    s = s_default;
    s.wn(RS.idx_ikpos_wn.qlim_par) = 1;
    if strcmp(Name_k, '3T2R')
      s.I_EE = I_EE_3T2R;
      % Calculate offset on the desired position to select dz in the mid of
      % the range for the 2T methods
      XE_k = XE_dz0;
    elseif strcmp(Name_k, '2T2R')
      s.I_EE = I_EE_2T2R;
    elseif strcmp(Name_k, '2T2R_limred')
      s.I_EE = I_EE_2T2R;
      s.wn(RS.idx_ikpos_wn.xlim_trans_hyp) = 1;
    elseif strcmp(Name_k, '2T3R_limred')
      s.I_EE = I_EE_2T3R;
      s.wn(RS.idx_ikpos_wn.xlim_trans_hyp) = 1;
    else
      error('Case unexpected');
    end
    for i = usr_ptno:usr_ptno
      % Run inverse kinematics algorithm (actual motion for paper results)
      comptimes = NaN(11,1);
      for jj = 1:11 % run 10 times to have valid timing information (first run for warm-up)
        t0 = tic();
        [q, phi, ~, Stats] = RS.invkin2(RS.x2tr(XE_k(i,:)'), q0, s);
        comptimes(jj) = toc(t0);
      end
      % Dummy-IK, um Werte für Nebenbedingungen zu 
      s_dummy = struct('finish_in_limits', false, ...
        'retry_on_limitviol', false, 'n_max', 1, 'retry_limit', 0, ...
        'wn', ones(RS.idx_ik_length.wnpos,1), 'K', zeros(RS.NJ,1), 'Kn', zeros(RS.NJ,1), ...
        'optimcrit_limits_hyp_deact', 0.9, 'Phit_tol', 1, 'Phir_tol', 1, ...
        'scale_lim', 0, 'normalize', false, 'I_EE', I_EE_3T2R);
      for i_dummy = 1:Stats.iter+1 % Laufvariable ist Anzahl der Iterationen pro Punkt -> Gütekrit pro Iteration pro Punkt
        q0_i_dummy = Stats.Q(i_dummy,:)';
        T_temp = RS.fkineEE(q0_i_dummy);
        [q_dummy, ~, ~,Stats_dummy] = RS.invkin2(T_temp(1:3,:), q0_i_dummy, s_dummy);
        if any(abs(q0_i_dummy - q_dummy) > 1e-8) % darf nicht sein (Logik-Fehler)
          error('IK-Ergebnis hat sich bei Test verändert');
        end
        % Write all information into the original Stats Variable
        Stats.h(i_dummy,2:end) = Stats_dummy.h(2,2:end);
      end
      % Phiz_trans berechnen (gleiche Rechnung wie in invkin-Fkt.)
      dz = NaN(Stats.iter+1, 1);
      phiz = NaN(Stats.iter+1, 1);
      for m = 1:(Stats.iter+1)
        T_0_D = RS.x2t(XE(i,:)');
        T_0_E = RS.fkineEE(Stats.Q(m,:)');
        xE = RS.t2x(T_0_E);
        phiz(m) = xE(6);
        r_limred2T2R_ist = T_0_E(1:3,4); % [SchapplerTapOrt2019, Gl. (1)
        r_limred2T2R_soll = T_0_D(1:3,4);
        phitz_norm_diff = -(r_limred2T2R_soll - r_limred2T2R_ist); % Soll-Ist und *(-1) damit nicht - in optimcrit nötig und Summe mit Inkrement einfacher ist
        R_E_0_wn910 = T_0_D(1:3,1:3)'; % Rotationsmatrix aus Soll-Transf.matrix
        Phit_z_limitvar = R_E_0_wn910*phitz_norm_diff;
        Phit_z_limitvar = Phit_z_limitvar(3);
        dz(m) = Phit_z_limitvar;
        % Debug: Check manually, if desired pose is reached
        tr_delta = T_0_D \ T_0_E;
      end
      Stats.dz = dz; % add to existing structure
      Stats.phiz = denormalize_angle_traj(phiz);
      % Store indices for residual
      RS.update_EE_FG([], s.I_EE)
      Stats.I_constr_red = RS.I_constr_red;
      Stats.I_constr_t_red = RS.I_constr_t_red;
      Stats.I_constr_r_red = RS.I_constr_r_red;
      % Store indices for converged steps of the IK
      Stats.I_iO = all(abs(Stats.PHI(1:Stats.iter+1,Stats.I_constr_red))<1e-3, 2);
      % Store timing information
      Stats.comptimes = comptimes;
      Stats_all{i,k} = Stats;
      % Warnung, wenn aktuelle IK ungültig
      if any(abs(phi) > Phirt_tol)
        warning(['2T2R: Inverse Kinematik bei Punkt %d für Optimierungs', ...
          'methode %d fehlerhaft!'], i, k); 
      end
    end % i (reference points)
  end % k (methods)
  
  %% Plot results: This creates Fig. 3 of the paper
  if usr_plot_convergence
  i = usr_ptno(1);
  sphdl = NaN(1,3);
  fighdl = figure(1);clf;
  sphdl(1) = subplot(1,3,1);hold on;
  figure_format_publication(gcf);
  leglinhdl = NaN(length(Names_Methods),1);
  leglinhdl2 = leglinhdl;
  for k = 1:length(Names_Methods)
    Stats = Stats_all{i,k};
    dz_iO = Stats.dz(1:Stats.iter+1);
    dz_niO = dz_iO; dz_iO(~Stats.I_iO) = NaN; dz_niO(Stats.I_iO) = NaN;
    leglinhdl(k) = plot(100*(0:Stats.iter)/Stats.iter, 1e3*dz_iO);
    set(gca, 'colororderindex', get(gca, 'colororderindex')-1);
    leglinhdl2(k) = plot(100*(0:Stats.iter)/Stats.iter, 1e3*dz_niO, '--');
  end
  linhdl_leg = line_format_publication(leglinhdl, lineformat);
  lineformat_dashed = lineformat;
  lineformat_dashed(:,3) = {'--'};
  line_format_publication(leglinhdl2, lineformat_dashed);
  
  % Plot limits (but do not add into the legend)
  xlim_dz = s.xlim(3,:);
  xlim_dz_thr = repmat(mean(xlim_dz),1,2) + repmat(xlim_dz(2)-xlim_dz(1),1,2).*...
                  [-0.5, +0.5]*0.8;
  plot([1;100], 1e3*xlim_dz(1)*[1;1], 'r--', 'LineWidth',1);
  plot([1;100], 1e3*xlim_dz(2)*[1;1], 'r--', 'LineWidth',1);
  plot([1;100], 1e3*xlim_dz_thr(1)*[1;1], 'm--', 'LineWidth',1);
  plot([1;100], 1e3*xlim_dz_thr(2)*[1;1], 'm--', 'LineWidth',1);
  grid on;
  xlabel('IK progress in percent');
  ylh1 = ylabel('distance $d_z$ in mm', 'interpreter', 'latex');
  [X1_off, X1_slope] = get_relative_position_in_axes(sphdl(1), 'x');
  [Y1_off, Y1_slope] = get_relative_position_in_axes(sphdl(1), 'y');
  set(ylh1, 'Position', [X1_off+X1_slope*(-1.45),Y1_off+Y1_slope*(-0.2), 0])
  ylim([-800, 1000])

  LegStr = Names_Methods_Leg; % [Names_Methods_Leg, 'lim', 'thresh'];
  leghdl = legend(linhdl_leg, LegStr);
  
  % figure(2);clf; figure_format_publication(gcf);
  sphdl(2) = subplot(1,3,2);
  hold on;
  leglinhdl = NaN(length(Names_Methods),1);
  for k = 1:length(Names_Methods)
    Stats = Stats_all{i,k};
    phiz_iO = Stats.phiz(1:Stats.iter+1);
    phiz_niO = phiz_iO; phiz_iO(~Stats.I_iO) = NaN; phiz_niO(Stats.I_iO) = NaN;
    leglinhdl(k) = plot(100*(0:Stats.iter)/Stats.iter, 180/pi*phiz_iO);
    set(gca, 'colororderindex', get(gca, 'colororderindex')-1);
    leglinhdl2(k) = plot(100*(0:Stats.iter)/Stats.iter, 180/pi*phiz_niO, '--');
  end
  line_format_publication(leglinhdl, lineformat);
  line_format_publication(leglinhdl2, lineformat_dashed);
  grid on;
  xlabel('IK progress in percent');
  ylh2 = ylabel('coordinate $\varphi_z$ in deg', 'interpreter', 'latex');
  [X2_off, X2_slope] = get_relative_position_in_axes(sphdl(2), 'x');
  [Y2_off, Y2_slope] = get_relative_position_in_axes(sphdl(2), 'y');
  set(ylh2, 'Position', [X2_off+X2_slope*(-1.43),Y2_off+Y2_slope*( 0), 0])
%   legend(leglinhdl, Names_Methods_Leg);
  
%   figure(3); clf; figure_format_publication(gcf);
  sphdl(3) = subplot(1,3,3); hold on;
  leglinhdl = NaN(length(Names_Methods),1);
  for k = 1:length(Names_Methods)   
    Stats = Stats_all{i,k};
    h_iO = Stats.h(1:Stats.iter+1, 1+RS.idx_ikpos_hn.qlim_par);
    h_niO = h_iO; h_iO(~Stats.I_iO) = NaN; h_niO(Stats.I_iO) = NaN;
    leglinhdl(k) = plot(100*(0:Stats.iter)/Stats.iter, h_iO);
    set(gca, 'colororderindex', get(gca, 'colororderindex')-1);
    leglinhdl2(k) = plot(100*(0:Stats.iter)/Stats.iter, h_niO, '--');
  end
  line_format_publication(leglinhdl, lineformat);
  line_format_publication(leglinhdl2, lineformat_dashed);
  grid on;
  xlabel('IK progress in percent');
  ylabel('opt. criterion $h_1$', 'interpreter', 'latex');
%   legend(leglinhdl, Names_Methods_Leg);
  figure_format_publication(sphdl);
  set_size_plot_subplot(fighdl, ...
    11.7,4,sphdl,...
    0.085,0.02,0.14,0.14,... %l r u d
    0.09,0) % x y
  for kk = 1:3
    set(sphdl(kk), 'xticklabel', {});
    axes(sphdl(kk)); %#ok<LAXES>
    xlh = get(sphdl(kk), 'xlabel');
    set(xlh, 'string', 'IK progress (0\,\%--100\,\%)', 'interpreter', 'latex')
    drawnow();
    [x_off, x_slope] = get_relative_position_in_axes(sphdl(kk), 'x');
    [y_off, y_slope] = get_relative_position_in_axes(sphdl(kk), 'y');
    set(xlh, 'Position', [x_off+x_slope*(0), y_off+y_slope*(-1.1), 0]);
  end
  set(leghdl, 'orientation', 'horizontal', 'position', [0.15,0.92,0.7,0.05]);
  drawnow();
  exportgraphics(fighdl, fullfile(paperfig_path,['ik_results','.pdf']),...
    'ContentType','vector');
  end
  %% Print additional information
  % This is used in the text in the paper in Sect. 4
  for i = usr_ptno:usr_ptno
%     change_current_figure(100+i);clf;hold on;
    for kk = 1:length(Names_Methods)
      Stats = Stats_all{i,kk};
      fprintf('Point %d: method %d (%s): %d\n', i, kk, Names_Methods{kk});
      fprintf('Number of iterations %d\n', Stats.iter);
      fprintf('Computation time total: %1.1fms (n=%d, σ=%1.3fms)\n', ...
        1e3*mean(Stats.comptimes(2:end)), length(Stats.comptimes(2:end)), ...
        1e3*std(Stats.comptimes(2:end)));
      fprintf('Computation time (per sample): %1.1fµs (n=%d, σ=%1.3fµs)\n', ...
        1e6*mean(Stats.comptimes(2:end)/Stats.iter), length(Stats.comptimes(2:end)), ...
        1e6*std(Stats.comptimes(2:end)/Stats.iter));
      Qdiff = diff(Stats.Q(1:Stats.iter+1,:));
      Qdiffabssum = sum(abs(Qdiff), 2);
      Qdiffmaxsingle = max(abs(Qdiff), [], 2);
%       plot(Qdiffabssum);
      fprintf('Initial change of summed joint angle sum: %1.1f deg\n', ...
        Qdiffabssum(1)*180/pi);
    end
  end
  %% Create Robot Figures. This creates Fig. 2 of the paper
  if usr_plot_robot
  for i = usr_ptno:usr_ptno
    for kk = 1:length(Names_Methods)
      for ii_q0 = 1:2
      Stats = Stats_all{i,kk};
      s_plot = struct( 'straight', 1, 'mode', 4);
      fhld_kk = change_current_figure(9);clf;hold all;
      set(fhld_kk, 'name', 'Rob', ...
        'color','w', 'NumberTitle', 'off', 'units','normalized',...
        'outerposition',[0 0 1 1]); % Vollbild, damit GIF größer wird
      view(3); axis auto; hold on; grid off;
      T_0_Di = RS.x2t(XE(i,:)');
      % Plot allowed range for the dz distance
      pts_ax = [eye(3,4)*T_0_Di*[0;0;xlim_2T2R(3,1);1], ...
                eye(3,4)*T_0_Di*[0;0;xlim_2T2R(3,2);1]];
      plot3(pts_ax(1,:), pts_ax(2,:), pts_ax(3,:), 'k--', 'LineWidth', 2);
      trplot(T_0_Di, 'frame', sprintf('D'), 'rgb', 'length', 0.4)
      if ii_q0 == 1, i_Q = 1;
      else,          i_Q = Stats.iter+1;
      end
      if ii_q0 == 1 && kk > 1, continue; end % initial pose identical for all
      RS.plot( Stats.Q(i_Q,:)', s_plot);
      view([66,11]); % manually found as a good view angle
      set(gca,'XTICKLABEL',{});set(gca,'YTICKLABEL', {});set(gca,'ZTICKLABEL',{});
      set(gca,'xtick',[],'ytick',[],'ztick',[]);
      set(get(gca, 'XAxis'), 'visible', 'off');
      set(get(gca, 'YAxis'), 'visible', 'off');
      set(get(gca, 'ZAxis'), 'visible', 'off');
      % increase line width of frames and annotations
      ach = get(gca, 'children');
      for iii = 1:length(ach)
        if strcmp(get(ach(iii), 'type'), 'hgtransform') % coordinate frame
          hgtrch = get(ach(iii), 'children');
          for jj = 1:length(hgtrch)
            if strcmp(get(hgtrch(jj), 'type'), 'line')
              set(hgtrch(jj), 'LineWidth', 1.5)
            end
            if strcmp(get(hgtrch(jj), 'type'), 'text')
              delete(hgtrch(jj)); % Put xyz label manually in InkScape
              % set(hgtrch(jj), 'Units', 'points', 'FontSize', 6)
            end
          end
        end
        if contains(get(ach(iii), 'displayname'), 'Link_')
          set(ach(iii), 'EdgeColor', [0 0 1], 'FaceAlpha', 1, ...
            'FaceColor', 0.7*[1 1 1]); % Blue links, grey Face
        end
      end
      name = sprintf('Rob_M%d_%s', kk, Names_Methods{kk});
      if ii_q0 == 1, name = [name, '_q0']; end %#ok<AGROW>
      cd(paperfig_path);
%       set_size_plot_subplot(fhld_kk, ...
%         6,6,gca,0,0,0,0,0,0); %w,b,axhdl l r u d x y
      drawnow()
      exportgraphics(fhld_kk, [name, '.png'], 'Resolution', '200');
      end
    end
  end
  end
  %% Create Animations
  if usr_create_anim
    maxduration_animation = 10;
    for i = usr_ptno:usr_ptno
      for kk = 1:length(Names_Methods)
        Stats = Stats_all{i,kk};
        Q_t = Stats.Q(1:Stats.iter+1,:);
        anim_filename = fullfile(respath, sprintf('traj_pt%d_M%d_%s_duration%1.0fs', ...
          i, kk, Names_Methods{kk}, maxduration_animation));
        s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
        s_plot = struct( 'straight', 1, 'mode', 4);
        fhld_kk = change_current_figure(10);clf;hold all;
        set(fhld_kk, 'name', 'Anim', ...
          'color','w', 'NumberTitle', 'off', 'units','normalized',...
          'outerposition',[0 0 1 1]); % Vollbild, damit GIF größer wird
        view(3); axis auto; hold on; grid on;
        xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
        T_0_Di = RS.x2t(XE(i,:)');
        trplot(T_0_Di, 'frame', sprintf('D'), 'rgb', 'length', 0.4)
        pts_ax = [eye(3,4)*T_0_Di*[0;0;-1;1], T_0_Di(1:3,4)];
        plot3(pts_ax(1,:), pts_ax(2,:), pts_ax(3,:), 'k--');
        t = (1:Stats.iter+1)';
        t_Vid = (0:1/30*(t(end)/maxduration_animation):t(end))';
        I_anim = knnsearch( t , t_Vid ); % Berechne Indizes in Traj.-Zeitstempeln
        RS.anim( Q_t(I_anim,:), [], s_anim, s_plot);
      end
    end
  end
end % robot
