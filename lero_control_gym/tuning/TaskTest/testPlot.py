from scipy.spatial import ConvexHull, convex_hull_plot_2d
from matplotlib import pyplot as plt
import numpy as np
Nominal = np.array([[11, 2], [11, 3], [12, 4], [16, 6], [19, 5], [19, 2]])
Rotor = np.array([[17, 5], [18, 5], [19, 6], [19, 9], [16, 7], [16, 6]])
Att = np.array([[6, 2], [10, 2], [11, 3], [10, 4], [7, 3]])
Wind = np.array([[14, 4], [14, 3], [16, 2], [19, 2], [19, 4]])
Pos = np.array([[12, 3], [13, 2], [14, 2], [16, 4], [15, 5], [13, 4]])

PSONom = np.array([[16, 3]])
PSORot = np.array([[17, 7]])
PSOAtt = np.array([[9, 3]])
PSOWin = np.array([[14, 4]])
PSOPos = np.array([[13, 3]])
PSO_all = np.array([PSONom[0], PSORot[0], PSOAtt[0], PSOWin[0], PSOPos[0]])
CPF = np.array([[5, 2], [19, 2], [19, 9]])


hull1 = ConvexHull(Nominal)
hull2 = ConvexHull(Rotor)
hull3 = ConvexHull(Att)
hull4 = ConvexHull(Wind)
hull5 = ConvexHull(Pos)

hull6 = ConvexHull(CPF)
hull7 = ConvexHull(PSO_all)


c1 = "aqua"
c2 = "orange"
c3 = "red"
c4 = "blue"
c5 = "green"

plt.xlim(0, 20)
plt.ylim(0, 20)
plt.fill(Nominal[:, 0], Nominal[:, 1], c=c1, alpha=0.4,
         label='Nominal Subspace', zorder=-1)
plt.fill(Rotor[:, 0], Rotor[:, 1], c=c2, alpha=0.4,
         label='Rotor Loss of Effectiveness Subspace', zorder=-1)
plt.fill(Att[:, 0], Att[:, 1], c=c3, alpha=0.4,
         label='Attitude Noise Subspace', zorder=-1)
plt.fill(Wind[:, 0], Wind[:, 1], c=c4, alpha=0.4,
         label='Wind Control Subspace', zorder=-1)
plt.fill(Pos[:, 0], Pos[:, 1], c=c5, alpha=0.4,
         label='Position Noise Subspace', zorder=-1)

for simplex in hull1.simplices:
    plt.plot(Nominal[simplex, 0], Nominal[simplex, 1], c=c1)

for simplex in hull2.simplices:
    plt.plot(Rotor[simplex, 0], Rotor[simplex, 1], c=c2)

for simplex in hull3.simplices:
    plt.plot(Att[simplex, 0], Att[simplex, 1], c=c3)

for simplex in hull4.simplices:
    plt.plot(Wind[simplex, 0], Wind[simplex, 1], c=c4)

for simplex in hull5.simplices:
    plt.plot(Pos[simplex, 0], Pos[simplex, 1], c=c5)

plt.scatter(PSONom[:, 0], PSONom[:, 1], marker="x", c=c1,
            s=150, alpha=1, linewidths=4,
            edgecolors="k", label="PSO Controller Set")

plt.scatter(PSORot[:, 0], PSORot[:, 1], marker="x", c=c2,
            s=150, alpha=1, linewidths=4,
            edgecolors="k")
plt.scatter(PSOAtt[:, 0], PSOAtt[:, 1], marker="x", c=c3,
            s=150, alpha=1, linewidths=4,
            edgecolors="k")
plt.scatter(PSOWin[:, 0], PSOWin[:, 1], marker="x", c=c4,
            s=150, alpha=1, linewidths=4,
            edgecolors="k")
plt.scatter(PSOPos[:, 0], PSOPos[:, 1], marker="x", c=c5,
            s=150, alpha=1, linewidths=4,
            edgecolors="k")

plt.plot([], [], "-", c="m", label="PSO Convex Hull")
plt.plot([], [], "-", c="k", label="CPF Convex Hull")

plt.scatter(CPF[:, 0], CPF[:, 1], marker="d", c="k",
            s=100, alpha=1, linewidths=2,
            edgecolors="k", label="CPF Controller Set")


for simplex in hull6.simplices:
    plt.plot(CPF[simplex, 0], CPF[simplex, 1], "k--", linewidth=3)

for simplex in hull7.simplices:
    plt.plot(PSO_all[simplex, 0], PSO_all[simplex, 1], "m--", linewidth=3)


font = {'family': 'normal',
        'weight': 'bold',
        'size': 10}

plt.rc('font', **font)

plt.legend()
plt.title("Quadcopter Roll/Pitch PD Controller Parameter Space")
plt.xlabel("P-Gain * 1000")
plt.ylabel("D-Gain * 1000")
plt.show()
