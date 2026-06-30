#!/usr/bin/env python3
import argparse

from planning_service_client.api.registry import RegistryClient


def main():
    parser = argparse.ArgumentParser(
        description="""Load planning problems data saved to a YAML file, use it to build PlanningProblem objects, and
        use them to generate regions. If the roadmap is not fully covered when all planning problems are used, continue
        generation from roadmap until full coverage. If no problems are specified, the Iris regions will be generated
        from the current roadmap."""
    )
    parser.add_argument(
        "--addr",
        type=str,
        help="Address of the IRIS builder service (defaults to localhost:5150)",
        default="localhost:5150",
    )
    parser.add_argument("--source", type=int, help="ID of the context to migrate from", required=True)
    parser.add_argument("--target", type=int, help="ID of the context to migrate to", required=True)
    parser.add_argument("--num_samples", type=int, help="Number of samples to use for repairing the regions", default=0)
    # add a flag --repair_artifacts that would default to False if not passed, true if passed
    parser.add_argument("--repair_artifacts", action="store_true", help="Flag to enable repairing artifacts")

    args = parser.parse_args()

    addr = args.addr
    source = args.source
    target = args.target
    num_repair_samples = args.num_samples
    repair_artifacts = args.repair_artifacts

    # Create iris client
    registry_client = RegistryClient(addr=addr)
    registry_client.migrate_planning_artifacts(
        source_context_id=source,
        target_context_id=target,
        num_repair_samples=num_repair_samples,
        repair_artifacts=repair_artifacts,
    )


if __name__ == "__main__":
    main()
