#!/usr/bin/env python3
import argparse

from planning_service_client.api.iris import IrisBuilderClient, SeedDataType


def main():
    parser = argparse.ArgumentParser(
        description="""Load planning problems data saved to a YAML file, use it to build PlanningProblem objects, and
        use them to generate regions. If the roadmap is not fully covered when all planning problems are used, continue
        generation from roadmap until full coverage. If no problems are specified, the IRIS regions will be generated
        from the current roadmap."""
    )
    parser.add_argument("--addr", type=str, help="Address of the IRIS builder service (defaults to localhost:5150)")
    # parser.add_argument(
    #     "--system", type=str, help="Name of the system (must match what the planning service is currently)"
    # )
    parser.add_argument("--context_id", type=int, help="ID of the context")
    parser.add_argument("--num_saved_problems", type=int, default=0, help="Number of saved problems to use")

    args = parser.parse_args()

    addr = args.addr if args.addr else "localhost:5150"
    # system = args.system if args.system else "no_system"
    context_id = args.context_id if args.context_id else 0

    # Create iris client
    iris_client = IrisBuilderClient(addr=addr)
    iris_client.set_context_by_id(name="spoon_scene", id=context_id)

    iris_client.start_build(seed_data_type=SeedDataType.ROADMAP, num_saved_problems=args.num_saved_problems)


if __name__ == "__main__":
    main()
