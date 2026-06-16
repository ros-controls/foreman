import os

import ctrlxdatalayer
from ctrlxdatalayer.variant import Result


def get_connection_string(
        ip="192.168.1.1",
        user="boschrexroth",
        password="boschrexroth",
        ssl_port=443):
    """Detects environment and combines a ctrlX Datalayer connection string."""
    if 'SNAP' in os.environ:
        return "ipc://"

    connection_string = f"tcp://{user}:{password}@{ip}"
    if ssl_port != 443:
        return f"{connection_string}?sslport={ssl_port}"
    return connection_string


def get_provider(system: ctrlxdatalayer.system.System,
                 ip="192.168.1.1",
                 user="boschrexroth",
                 password="boschrexroth",
                 ssl_port=443):
    """Creates and starts a ctrlX Datalayer provider instance."""
    conn_string = get_connection_string(ip, user, password, ssl_port)
    provider = system.factory().create_provider(conn_string)

    if provider.start() == Result.OK:
        return provider, conn_string

    provider.close()
    return None, conn_string
