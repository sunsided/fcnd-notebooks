from matplotlib import pyplot as plt


def compare_planned_to_actual(z_actual, z_path, t, additional=None):
    plt.subplot(211)
    plt.plot(t,z_path,linestyle='-',marker='.',color='red')
    plt.plot(t,z_actual,linestyle='-',color='blue')
    if additional:
        plt.plot(t, additional, linestyle="-", color="black")
    plt.grid()
    plt.title('Change in height').set_fontsize(20)
    plt.xlabel('$t$ [sec]').set_fontsize(20)
    plt.ylabel('$z-z_0$ [$m$]').set_fontsize(20)
    plt.xticks(fontsize = 14)
    plt.yticks(fontsize = 14)
    plt.gca().invert_yaxis()
    legend = ['Planned path','Executed path']
    if additional:
        legend.append("Executed path (FF)")
    plt.legend(legend,fontsize = 14)
    plt.show()

    if additional:
        return

    plt.subplot(212)
    plt.plot(t,abs(z_path-z_actual),linestyle='-',marker='.',color='blue')
    plt.grid()
    plt.title('Error value ').set_fontsize(20)
    plt.xlabel('$t$ [sec]').set_fontsize(20)
    plt.ylabel('||$z_{target} - z_{actual}$|| [$m$]').set_fontsize(20)
    plt.xticks(fontsize = 14)
    plt.yticks(fontsize = 14)
    plt.legend(['Error'],fontsize = 14)
    plt.show()
