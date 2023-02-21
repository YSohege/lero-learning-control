import socket
import torch


class Networked_Underwater_ROV:
    """
    A class for communicating with a remote Networked_Underwater_ROV environment over UDP.
    """

    def __init__(self, ip_address, port, timeout=2.0):
        """
        Initializes a Networked_Underwater_ROV object with the given IP address and port number.
        """
        self.ip_address = ip_address
        self.port = port
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout( self.timeout)


    def __del__(self):
        """
        Closes the UDP socket.
        """
        self.sock.close()

    def send_action_receive_state(self, action):
        """
        Sends an action to the remote gym environment and receives the updated state.

        Parameters:
        - action (torch.Tensor): a PyTorch tensor representing the action to be sent to the remote gym environment

        Returns:
        - state (torch.Tensor): a PyTorch tensor representing the updated state received from the remote gym environment
        """
        # Send the action to the remote environment
        message = action.numpy().tobytes()
        self.sock.sendto(message, (self.ip_address, self.port))

        # Receive the updated state from the remote environment
        data, address = self.sock.recvfrom(1024)
        state = torch.from_numpy(bytes(data)).float()

        # Return the updated state
        return state


def main():
    # Instantiate a GymEnvironment object with the IP address and port number of the remote environment
    env = Networked_Underwater_ROV("192.168.0.2", 1234)

    # Send an action to the remote environment and receive the updated state
    action = torch.tensor([0.1, 0.2, 0.3, 0.4])
    state = env.send_action_receive_state(action)

    # Print the updated state
    print(state)


if __name__ == '__main__':
    main()
