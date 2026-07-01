%% flipPlantCheck.m
clear; clc;

% Parameter
p.m = 0.063;            % kg
p.J = 7.16914e-5;       % kg·m² (Jyy)
p.g = 9.81;             % m/s²

tspan = [0 2];          % simulation time
s0 = [0; -1; 0;  0; 0; 0];     % 초기 조건 : x=0, z=-1(고도 1m) 
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);

%% ① 자유낙하 (T=0, τ=0) — z̈=g
T = 0;  tau = 0;
[t1, s1] = ode45(@(t,s) flipDynamics(t,s,T,tau,p), tspan, s0, opts);
z_exact = s0(2) + 0.5*p.g*t1.^2;          % 해석해: z = z0 + ½gt²

%% ② 호버 (T=mg, τ=0)
T = p.m*p.g;  tau = 0;
[t2, s2] = ode45(@(t,s) flipDynamics(t,s,T,tau,p), tspan, s0, opts);

%% ③ 등각가속 회전 (T=mg, τ=const) — θ=½(τ/J)t²
T = p.m*p.g;  tau = 2e-4;
[t3, s3] = ode45(@(t,s) flipDynamics(t,s,T,tau,p), tspan, s0, opts);
theta_exact = 0.5*(tau/p.J)*t3.^2;        % 해석해

%% --- 플롯 ---
fig = figure('Name','Phase0 plant sanity check','Color','k'); %[output:27ebd8b2]

subplot(2,2,1); %[output:27ebd8b2]
plot(t1, s1(:,2),'w', t1, z_exact,'y--','LineWidth',1.3); %[output:27ebd8b2]
xlabel('t [s]'); ylabel('z [m] (NED, down+)'); %[output:27ebd8b2]
title('(1) free fall'); legend('ode45','½gt² exact','Location','best'); grid on; %[output:27ebd8b2]

subplot(2,2,2); %[output:27ebd8b2]
plot(t2, -s2(:,2),'w','LineWidth',1.3); %[output:27ebd8b2]
xlabel('t [s]'); ylabel('altitude -z [m]'); %[output:27ebd8b2]
title('(2) hover: altitude hold'); ylim([0 2]); grid on; %[output:27ebd8b2]

subplot(2,2,3); %[output:27ebd8b2]
plot(t3, s3(:,3),'w', t3, theta_exact,'y--','LineWidth',1.3); %[output:27ebd8b2]
xlabel('t [s]'); ylabel('\theta [rad]'); %[output:27ebd8b2]
title('(3) const-\tau spin'); legend('ode45','½(\tau/J)t² exact','Location','best'); grid on; %[output:27ebd8b2]

subplot(2,2,4); %[output:27ebd8b2]
plot(t2, s2(:,3),'w','LineWidth',1.3); %[output:27ebd8b2]
xlabel('t [s]'); ylabel('\theta [rad]'); %[output:27ebd8b2]
title('(2) hover: \theta stays 0'); grid on; %[output:27ebd8b2]

% --- 검은 배경 스타일 일괄 적용 ---
ax = findobj(fig,'Type','axes');
set(ax, 'Color','k', 'XColor','w', 'YColor','w', 'ZColor','w', ... %[output:27ebd8b2]
        'GridColor',[0.8 0.8 0.8], 'GridAlpha',0.3); %[output:27ebd8b2]
set(findobj(fig,'Type','legend'), 'TextColor','w', 'Color','k', 'EdgeColor','w'); %[output:27ebd8b2]
set(findall(fig,'Type','text'), 'Color','w'); %[output:27ebd8b2]

%% --- 콘솔 정량 검증 ---
fprintf('--- Phase 0 sanity check (NED) ---\n'); %[output:4ed420cc]
fprintf('(1) free fall   z 오차 : %.2e m\n',   abs(s1(end,2) - z_exact(end))); %[output:407a606e]
fprintf('(2) hover       고도   : %.4f m (1.0 기대),  θ=%.2e rad\n', -s2(end,2), s2(end,3)); %[output:06ad9e52]
fprintf('(3) spin        θ 오차 : %.2e rad\n', abs(s3(end,3) - theta_exact(end))); %[output:5f3fd977]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":20.3}
%---
%[output:27ebd8b2]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAJAAAABXCAYAAADrq8y0AAAAAXNSR0IArs4c6QAAGZxJREFUeF7t3QV0XNUWBuAT3N21uLsVd7fiDkWLu5XiTrHi7g7FXYu7uxSKl+JWiuet77x38m4uY5FJE2b2WllpZ64c+c\/2vdMQQmgMFdCpp54a9txzz7DooouGp59+utkdX3\/9dfj111\/DVFNN1ezzs846K+y8885hgQUWCC+++GKz75ZZZpkwYMCAcPLJJ4d99903rL\/++uGGG26I1\/Tu3Tscf\/zxYaONNgrXXntt\/Oy+++4L++yzT3zPH3\/8EX9\/8cUX4Z133gmzzDJLOO6448LFF18cr\/3www\/Dn3\/+2fTMQw45JIw00kjhsMMOC4MGDQrTTDNNePjhh8Pyyy8fr+\/Ro0fYfPPN4\/UvvfRSmG+++eLnDz74YHj\/\/ffDDjvs0DT2nj17hssuuyxMOumk8f1PPPFE8FmWvL9fv35hl112CUsuuWR4\/PHHK1jhEB544IE4piWWWCI+13oY00ILLRTn9NVXX8UxrbDCCmHBBRcMzz33XLDGu+66a5hooolKfm8AQ4YMCSOPPHKYYIIJ4nhOPPHEuPbbb799uP322+N8nn\/++fg+ZG\/23nvvZt\/n59tQKYD69OkTjj766LDWWmvFl40zzjhhpZVWii+ycTa1V69e4a+\/\/go333xz\/Py6664LG264YZh22mnDxx9\/XDGATjnllAgWwDOhxsbGcNVVV4XRRhstLq6F22CDDcKdd94Zll566QiOk046KX737rvvhq222iq+K4GyEIA8o3v37nG8n3\/+eRhzzDHjwnoXYC+33HJxs4444ojw6KOPxuf9\/vvvYcIJJww\/\/\/xz\/L8NB5BtttkmOESHH3540\/vPPPPMggACJkBYZ511\/gGqp556Ko5ps802CyOMMELo27dvmHzyyeNzbrnllvDZZ581AWiOOeYIr7\/+egS8vTGfLMDy3999993xXvM68MADw8CBA+OadevWLR6QCy64ILzwwgth\/vnnj4wCmByCySabLALowgsvLDjfigG0+uqrhzvuuCMukkWdc845w2uvvfaPRfjtt9\/iRiODHG+88eLpyFMxDmQDH3nkkbDsssuGNdZYI4K1EI0\/\/vjxPU7E9NNPH3755Zfw2GOPhd133z1uYjkAAb9FwzWHDh0aRh111Pjz999\/h5VXXjk89NBDccFXXHHFeHInnnji8Morr4R55523aTgW1yLjGmks6f3FAIQzfvrpp3Hj8mRzjWmKKaYI9957bzjyyCPDrbfeGsYdd9y48UCSONCII44Yv\/P5Rx99FIGcBVD++9lmmy3MM8884fLLLw8zzzxz5Io33XRTOPvss8Nuu+0WjNeeXnrppQH4SBljwNlxOJyu0HwLAsiJ3WmnneKCEh0Q2NDQEFFpgZ0OLHW77bYLTk0hMuA333wzXHnllWGLLbaoiIW7iGhxCm2sU12KcJ4kroDI2KpFs88+ezj33HPjJliH\/fbbL9x2223Vel2Xee4\/ALTeeutFUeVU\/fjjj1EcOdlOAxbqdPjOqTj99NPj6Se+8nTeeedF1rjIIouEZ599tuIFwdqdDPqKd5aiTTbZJFx99dVh\/\/33j\/K8UjIe1yedK933xhtvxNNXiHBbbPyMM86Iotu99CCcr5bpHwCinI0yyijhySefjOuy1157RblIJ\/nggw\/CsGHDwieffBIVLRtBySJyskQJxW7pLUkfackiY9OAh4t5VyGis7z11ltRmZ5rrrkC0Vkp0WFsfP\/+\/eP80r1ES94Q8Eycbuuttw6XXHJJVM7R999\/H3U0SnYtU1kdiKJKgSOOzjnnnDD33HM3rdc111wTdQVyuysRoCy88MKRe9HRKPr0pmIAys\/NvTfeeGM0DqopNrvCmjY00lrrNFxWgL7HWOjKVJYDdeXJFRt7ntPwq1D2J5lkkmggFCPcl37GzGWRZonpz7LsitQWINckgDhD89YjE3XLLbeMvpdCxFhgkdLpuA7yhJGzVKtBSy21VJMvqhrPb8vYaxJA2U1YddVVox5XTgnHYZjxnKOFqC2bUA4UdQCVW6F2+p5VZZN5iMsRJ9xPP\/0UVllllbDmmmvGe4Q5EnFVcCCi6aabLlqgeZBxIySvex1A5Va8C3xfKYA23XTTGB4AGAACBI7BFJppzVTrAGrNqnXgPYKrPNRIzIZfin+Ga168iL9o6qmnDsccc0zkJkIuiy22WODSF4Lgr0Kcf7jGUUcdFWNOfFZiSf4tvtZaqlkADRo0qFOa8cRGIpxBADBFzzkaeZJHH330GMgUMxt77LGjmMFNbOaOO+7YBAg+G15kAcXrr78+xq84S4ViROHXXnvt6PX+4Ycfol8IJyvnBa8r0f9dgU7rB8paNGJzzGtBXCTMkWJyfuMm6K677goi+TgVYKUMAGGX888\/P3zzzTfRg05cMbkTgAQXRfJ5zsX\/7rnnngjQpN9UwpVqlgN169atU3IgQdJEeQART+JvclukYyQAiR7beAASHT\/44IOb7T0LaoYZZojhCOkoIuxSUQ444IBm1x100EGRQ3lPluhOYnzCGrhUlmoWQJXmA1VyCqt1DdEFRLy2OBOTGmjk70jmWm211eK\/xaXoMUBFTAk5sJxwLklgRFyiLAcCDLG3PfbYI4ISEAWKhXAS0aGY0\/QtIK0D6H8irCsAyFApx4svvngEkFwWii9xI54lQ4ASLcB62mmnRYcfMcfHA0yUaDkv\/p0FkM88V1D2oosuivkwOIn4Hy6UJXlAnnP\/\/fdHV0EdQF0MQO3J3UTcr7jiiqgHiapLThNpZ9HhbqWIhVcH0P9XqCY90eJYXAEUblxFdiAlWt53ygcuBqJSAKIb0d1StqFoPZIx2JbPiU7PaOtzio3H4WltGKYmAfT222+HWWedNW5udmOl4M4444yt5kDtySU7+ll1ALVgxQGIgi3jMgGIH0kCvwqP1oowHCgLyuyJb8vnWQ7UlucUG0+dA7UAPC494YQTYjI6XebYY4+NXmqOR1F2DsvWAqi1p7jc8OvB1HIrVOZ7dVx8Mjaa0puIn0ftkjTbUiTjkAeaJSUGhuR380BL4xg8eHD8nk6kdKcQ0ZnkSyffEwemYoGUV133A7Vxk6t5Oz8PEOAOKiKQIKicZGZ4KSKamOVOsWfwVoujsbZSvVd7jL1mAdStW\/PK1IwDuGldC5QwFV3zatzvZcQNLgRAYmD8MCo4EscQWFXTRM6Lso8xxhgxfCFJDADlb\/turLHGioryq6++2iyf2ec4jGIAXmwVmxyTzzzzTKwN42cCRP4i5Txia6wtHEnoQ+ysJkVYY2NzABVKqmtJ1nQ17s8DSNmRwjdBViEJpTbEnNiWAkhReSU66667bgQc8DHT\/U7VrpyMcoLExzgiKcAAqLpWlQeA8E57nrgav5FSa8Fa3mjcjNMSaHi4gakmATRoUHMAZYLgTVwmk2dVlttX43719SLuSpBtOt1FqAKnEBPbdtttIxD8m1KMlBoJd2QBhHu4R0jEM1OoQqCWHuXn5ZdfjvfTjZKpL1+a1SY9RLGjOBkQ42Q4ICBzRtYkgLpKKCMhV4Qcl\/jyyy9j6EJJLuVXHTwAyBtCdBz6URZA6Rk4hhznRACkvPm7776L3ChPYmO810Sm8h8Wm0R8cTe6jwYPIvp1AJXlL8P3AoFT8S8RcXk76tHErN57770gIKqq1g8RppIUqHAtgdSshxmXEZkXuvj2229jIFZ8TfIZUUgX0kRCXbiybuXMAClyT9cCNvE2uUTicwAGzHUADV98lHw74IiQO\/1OvQxCoNFQACCAiikODLgEEcTfo3pWBwteZvoMogcBVbaMmUMRmCSVEXMsNflEgqfSQyjQxsBsl0ttLEq+VXcAHguxDqBODKByQ5PbI+1DPyFcgpWly0ax0mjPowzrHAI4OFWxGkv1Yiwyek4qbeYXypZ116wZP3To0EYynChgjtIxLLzTma1SKLeB1fzeyXba1coLNQALDsCyUhtPGZZlKKOQyU3RxX04GQuRdiWIBUdPAjbiECdhWeWJmCpX1p3AR8nXsKo9qZQnmshmLHBR6GeQuHJL3p8HP90OJyamy1FDnz59Gvk8sGemKB8Ldm9T2pJkXu7FLfk+deHQCMEGE1FJ2RWhBiymvN48gMDs5nUu1L\/Ie1PZD2tKXx3ebWJLzjTTP09ACWgssUTiR4Ango9YfTa6K1K2MpVfK+mNcFGOOm1OdLmB\/xu+1yFEdqOU2plmmin6sSTCObj5hLVC8+XkpOspQBBaERwGBgZAKSK6Wa\/yoRwcOqLDSdwL6yQimbgyPI\/bwhhZtYwLTloAa+jevXsjBxqXv4spiE4wnwn\/Suo76KEUSqLEA0zSqaS8+r9OV04wZ13iZl4owu2kHnroobFNjPRSegqu4lomOQvIs1g8OIDJ8PNw3mHRuCLHIUvHouMaPMycfRaRwy+1xTNOyi+xR1ThHuJfNknogsJN7KEUB6OIq3n3buLRO+hTxsrHI2NRLjWOhSjOOKCN8JmWOGJrafE5JXEjv7kWvNtYmPqpJ6SMgCmnnDKKHVUjMis33njjCCI9IssltqViR\/vEsAAEYocKwvosRdbVXO0h0Nm7\/LoajzJv\/jWqDJBJ+bXX5iZ703eRA7nA4FMjKOzfDVAq7TMRE9fLLRil0gL5v8kwq+Ufa0DAxwJoNsDm2QTgQ8Dk5UIHNh2A6Ax0h9Q40qJYSAvL9wIM3qNhVdp8lpGNdPL4dehAdCJEh\/MOCrJ3ABsApJxoSnYCkFNnM7kItNMDIOQQ0X0cIuMXuLVwAGGzgJS3W5jEIZNiC0DSZinbNhIIWXue7VkOijY59EvvpM8BYvbUV8pZ1eg7+HlyEPPFBPlr0rpyutJz7Ht+XRPQMQPz4brAXBxuawovXCENPXv2bIQs3IKn1Qbzo1igxClKTcpDodhpypOYkbJhC5X6FgpmCiM4jUxlAFLchzPw7PLpUIb5WDjvZA\/ywyDvsUBOBpDZZPIaZ8O9UnVG8u3gqESB51iERIWsLV5sprjTbC1QUi6FNixkavgJ0E6wzx0SY+YdBw6cyPtwFk0ceKo9m7MSJzVea5U4EM7fGjJXQJER4FnWyz4KMvNLlSLjs34AznDIrms6mAlADA1SJnWlMw8H0Ht56GNGov9wqllA1ksCkAAkrb4UGbzTTRH1YEV\/RAWOgivgQLy2PkdOrY3FgSigpQDkemyWfwen0CFVkJQ8TqfMM1LVaRpnAhduqL0wbij\/J1HKOsRtbDRuSjwSRw4Eiwbnc0Ip6ECLG+MUTqrrEgcFOORznIlItQmpZS+lHuA902c4E\/BZNyIU5y5Fxoj72nRcNZE1wPWAESiBhuSwd8VSUrLvAQRryfiwx+Zkzxgpmk0kADkg1A9rTK1grXt3E4B69+7dSBQ5dUnU4AgWPlk2pSaIfbsfAFluUGlwJmwyTi2fiUFg48pz3EM0JB2oGAeyuUlJZFElKzFxBxYDYFk8egrWiiwExQ+AcFQ\/LLTUik\/HUcBOOpCWd7gkEZOCou7HPXE23JNy60AhEXstcs3DJuBIDhAwGKf3stKILuoBcawdsTWQtEbUFgKQ67I6EO5E98OFOUGzAEq9JOlWxCY9RmNU3KIcWVcHxsH0Pla38fkcF\/UcYg1YcDTKshARJuDAGgeAyY5oGDZsWCNTGPL4OpCHJnPYQpQiICCO6D02hEeXvOTtRUAJsbqcUlaJFJtBaSsHIDqRk09JpuvQsVgtNiJP3okrIKByPd3M4uOA9BsKoGcCMRGAPeO4FpA+YcG8g7gjdohxVg1A0FuABbeyqSmVBEexuZyNiA8K10hzk7SP4+FQOGji6oUAZEOyVpj5WFM\/npcFkENqvxgjYng2lG+sErIGrvfbc0gF77WP9o3uSN9z8LLJeNQMeii9jhQR4G7o379\/Y76TOt3ChmO7ZHopohybCCsGOclYYCGdqJLJFbrGhtGDnGbjKkWuM1GnKd+FlREAjBYu66HWsQzbdz2FHifN931M3Mp3qUl4R8zdXFMqShZArV3LUvelfpgt6XnZMHjw4MZsxy2njJJrMaGvHLmXjkA5RkAHuerR24uSI5EPgulcipL+Y3Mph4WIuKb4ImZ+GqtTyEWAa+abZyYAZT3NHTH3jgJQqbkXW2+GRUNDQ3NfIpbMoqAYVxLKYAbSBdLftMDu+YPakwPRMYgC3MdGFmtATpeic\/GHpL8HkZ08lktfYuYnSn9rgq7DpKYLEId5SgASCwNoorgj5l5tAFlb6otDxX9UaO7FAGSNC9aFUX5LBSGzD6RocyilRgTJGUbvaE9qCRsvNn76GZDxcSViOVJWmbPmkUpf8mMvVBvfGefe0jWnCznsrKpCDePLPa9h4MCBsTsHZSqbZFXuxvR9aotC6SIynHCWAFHSntQSAFGKSxFzOzUUx3EAgS+Ja79Ye7xCtfGdce4tWfPs3FOWQUvud21Dr169ohnPq5gCg8UeQpl1gpMjjumKtfNO8iHxJbBmnNb2aj9NYWW5YLXMdsquGE1yLhYaK9MTSSLDCVkNxB79yVxxmaRL+U5ukQU0fs9nlSQrMv\/8fGlzZ5t7SwCQnXu6r9TcCz27HkxtyYoPp2vb0se52kP+V9fG82+lSHVaSLoOs72cNVds4Ys1V6g3Gq82VIfD81mDHHhEYIqPiRlxVbRWya+3d2m+kf9qDsQjLJYDLLzAPLbCGeJorYmAW7o6gLoogCrtAW16qYk4biOMwIeU9WkJjwgNtITqtfGFV6vLcKBKAZRtIs63wSIU8U+JceJLcpza01NuaWs2qb6ju7Rmu68mTFeriTi9B4joQdzuouj8OaLLUiTak2oWQB3998IAqKOaiHOUSQ+Vl+Sd3u0vHYuDif20J9UsgAYMGNDhfaKzf2Stmk3EeYoFRv1O5chKl3ihRebbk2oWQMO7Nr6aTcTlHAnyyh+SLcDTLqlNY4W6DvT\/49MW8A93JbqaTcSJKdkFUmxxHjlLwiGSy8qVvuS5k+wErn+hnEJ\/8rwtm1COE9Zb3JVZoWo0EZdVqZyGIp2aiAsYAxMO1FKSVy1QLINRWkj+T57XAdTSFe3k1wOLACoHYlsDu\/Xa+OKbPdxFWDVxyPqSuqFITj+hLJAk0ldKldbG1xuNV7qiXeQ6SrRarELUklYsldTGt5XLDe8lbcl6ZMf6r+ZASnRUWRTa3NTttZKNU3DJk51a3rmHQi5YmxLQvaPeaLyS1azCNVJQWUyCnNmNVWkpuV\/hXCLFdJK+lPfkid6jrklNk3JmFpjqUIlw\/ta7ilcFAynZXoGf2nu1VYr\/1EpJ8RT3ypIyF6khfqee1MpyKOqpoqWuRFcBGJU+Ul2VRH6bmNJRZRwqcEuVn54lYV+6LFAUIu1XAAXHSPfxRvvRhpdTUe2TYCpSaSm1A5BkJMqPVvDovXkq1FhCInpKBa0DqNLdrtJ1NtPGAhBOIqKugQAzPJH4FR+OikpVn0QG0EjR4CjkHFQMkO1r4zM\/qWOHTVcc6DPpuPQkPh6\/VXeqOBW9z1NqLJHq3VXCKl8CVL4hJc2t1SPKLWmn9gP17Nm8zW+xyRRoBBEv7dmz3PT\/+32x+9PdWQCJlivNoXPIHCSKNGog3tSqy+tBwEOU4TCIj0fJTbYYUpWIn9Q1jEWmO4Ufz\/MOudDAw2orVcokJCK2xg+E4yioJO60malZDpRvNF4MDoUaiLu20ibkxe73DDVZ2qTYUGDxf8nqNlfxHu5C\/PDrZEMQ0jRc41plyjgRbpClPICUNzPphTUSgDQQBwplPapagbkQpe4dKaNAaoixCZXULIAuuaQyDvS\/v2T0j3UtUzrfdH2x+\/MPxCEoyjiB\/GUchqKqxt3GZ7tZSApTwqyxAw6D1cv\/USCIgymSU3Hhu9SuD4CGDBkSQZIAlP0DLqX4Kc+2LMdCrd9qFkDDO5ia3TAlPHQUynQqEVKzpZEUkoZKhKUNx5103SBGiBN6k5iXHGjdRYBP5ammDqnwUfcQ9e0S1Fh3OBZAVUIcilq7KfmRYwRI+gB4Vx1AlaxgFa9hAdFDxKlYNvKYnXj6DNFls3AaIKG\/IGDQzEHDBG1V1IFRhvl\/ROGV3lLKZSkCHY6mS5kiQuJKcBQYcCdKcTnyHN1KKOTA7DdxKz1FY4KaVKI7Ewcqt4FMexwltbIrd33+e76hrInfkvvrf\/a78Gp1OU90KUdiKUCoRqVfZR2JLQFQ\/c9+\/0sA1JJNr8a19bKe5qva5ThQJaBgqWlhp6uYsIewQ\/q7Fvm+P5U8L3tNvTI1B6COTqpv6YbVrw+xzV42j7wzrUlDjx49Gpm7goiliu0qCSh2lompvMi2g+McZKGJuVUyV2mvEtGQaH7yfPt\/MQ7UWebe0eOIIkyDIc0BBBpLUbmAYkcPvtj7AEgXshTSACBKMKp0rsWeXQdQAR2I\/0KtVOqfXGzxCnWqz6c+dAYQMdelhwgxcBpyOCYAVTrX\/DzKlTZ3hnkPjzE09O3bt1HXUk4xEe7WkACjeBTWzwssGKp3cXtRsWbbpZ4vPUSjcJF2jkmORZH9QnNVWiSFQ\/Wqvoecg5WmvHbGubdk3dsyd+9p6NevX6PTZYFb0yNPWY6qB+3hbJCYlWfxKrcHlWq2Xcnz\/TkCMTLcFffJz1VCmT7W5iHRTDCXxSZ\/qBx19rmXG39b5p6e3WYzHlBwr\/THbnV357VtL6uhVLPtcgtUyfc4kriboCzCSYU61NSXo1qee7sBqJabbdfy3NsNQLXcbLuW595uAKrlZtu1PPd2A1AtN9uu5bknAP0HPvGQ70r7KR4AAAAASUVORK5CYII=","height":87,"width":144}}
%---
%[output:4ed420cc]
%   data: {"dataType":"text","outputData":{"text":"--- Phase 0 sanity check (NED) ---\n","truncated":false}}
%---
%[output:407a606e]
%   data: {"dataType":"text","outputData":{"text":"(1) free fall   z 오차 : 3.55e-15 m\n","truncated":false}}
%---
%[output:06ad9e52]
%   data: {"dataType":"text","outputData":{"text":"(2) hover       고도   : 1.0000 m (1.0 기대),  θ=0.00e+00 rad\n","truncated":false}}
%---
%[output:5f3fd977]
%   data: {"dataType":"text","outputData":{"text":"(3) spin        θ 오차 : 0.00e+00 rad\n","truncated":false}}
%---
