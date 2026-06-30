#!/usr/bin/env python3

import argparse

from planning_service_client.api.registry import RegistryClient
from proto.basic_types_pb2 import PlanContextId


def main():
    parser = argparse.ArgumentParser(description="Process a planning problem saved to a YAML file.")
    parser.add_argument(
        "--addr", type=str, help="Address of the meshcat visualizer service (defaults to localhost:5550)"
    )
    parser.add_argument(
        "--system",
        type=str,
        help="Name of the system (must match what the planning service is currently)",
        required=True,
    )
    parser.add_argument("--num_samples", type=int, help="Number of samples to use for evaluating the artifacts")

    args = parser.parse_args()

    addr = args.addr if args.addr else "localhost:5050"
    system = args.system
    num_samples = args.num_samples if args.num_samples else 0

    client = RegistryClient(addr=addr)

    resp = client.get_plan_context_summaries(system)
    ids = []
    for summary in resp.summaries:
        ids.append(PlanContextId(value=summary.id.value))

    artifact_resp = client.get_planning_artifact_status(context_ids=ids, num_samples=num_samples, system=system)
    print(artifact_resp)


if __name__ == "__main__":
    main()
