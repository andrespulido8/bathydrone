from tkinter import N
import numpy as np
import matplotlib.pyplot as plt

mph_to_mps = 0.44704
N_to_lbs = 0.2248089


def main():
    altitude_feet = 20
    rope_length_feet = 31
    alpha = np.arcsin(altitude_feet / rope_length_feet)

    speed_x_mph = np.array([3.5, 6, 9, 12, 15])  # miles per hour
    n = speed_x_mph.shape[0]
    speed_x_mps = speed_x_mph*mph_to_mps

    total_force_N = np.array([9, 12, 18, 27.5, 31])
    force_x_N = total_force_N*np.cos(alpha)

    lambd = force_x_N
    H_x = np.stack((speed_x_mps, speed_x_mps**2), axis=1)

    lambd2 = force_x_N
    H_x2 = speed_x_mps
    drag_x2 = np.linalg.lstsq(H_x2, lambd2, rcond=None)[0]
    print("drag coefficients 2: ", drag_x2)

    drag_x = np.linalg.lstsq(H_x, lambd, rcond=None)[0]
    print("drag coefficients: ", drag_x)

    F = H_x@drag_x
    plt.plot(speed_x_mps, force_x_N, 'o',
             label='Original data', markersize=10)
    plt.plot(speed_x_mps, F, 'r', label='Fitted line')
    plt.xlabel("Speed in x direction [m/s]")
    plt.ylabel("Force in x direction [N]")
    plt.title('Bathydrone relationship between tension force and speed')
    plt.legend()
    plt.show()

    plt.plot(speed_x_mps/mph_to_mps, force_x_N*N_to_lbs, 'o',
             label='Original data', markersize=10)
    plt.plot(speed_x_mps/mph_to_mps, F*N_to_lbs, 'r', label='Fitted line')
    plt.xlabel("Speed in x direction [miles per hour]")
    plt.ylabel("Force in x direction [lbs force]")
    plt.title('Bathydrone relationship between tension force and speed')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
    print("Done")
