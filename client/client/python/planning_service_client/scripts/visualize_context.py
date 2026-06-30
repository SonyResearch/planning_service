import click
from planning_service_client.api.base_client import ClientOptions
from planning_service_client.api.visualizer import VisualizerClient


@click.command()
@click.option("--addr", type=str, default="localhost:5550", help="Address of the meshcat visualizer service")
@click.option("--id", "context_id", type=int, required=True, help="ID of the context to visualize")
@click.option("--show_prm", is_flag=True, default=False, help="Display the PRM (default: False)")
@click.option("--show_iris", is_flag=True, default=False, help="Display the IRIS regions (default: False)")
def visualizer(addr: str, context_id: int, show_prm: bool, show_iris: bool):
    # Create visualization client
    options = ClientOptions()
    options.log_level = "CRITICAL"
    print("Starting visualizer client...")
    with VisualizerClient(addr=addr, options=options) as vc:
        try:
            vc.visualize_context(
                context_id=context_id, force_reload=True, show_prm=show_prm, show_iris_regions=show_iris
            )
            input("Visualizer started. Press Enter to quit.")
        except KeyboardInterrupt:
            pass
        print("Exiting visualizer.")
        vc.stop_visualizer()


if __name__ == "__main__":
    visualizer()
