plot_non_lin=true;
if plot_non_lin
    figure()
    plot3(out.ref.Data(:,1),out.ref.Data(:,2),out.ref.Data(:,3))
    hold on
    plot3(out.pos.Data(:,1),out.pos.Data(:,2),out.pos.Data(:,3))
end
plot_lin=false;
if plot_lin
    figure()
    plot3(out.ref.Data(:,1),out.ref.Data(:,2),out.ref.Data(:,3))
    hold on
    plot3(out.pos_LTI.Data(:,1),out.pos_LTI.Data(:,2),out.pos_LTI.Data(:,3))
end
