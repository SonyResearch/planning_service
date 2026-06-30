#!/usr/bin/env python3
import argparse

from planning_service_client.api.iris import IrisBuilderClient


def main():
    parser = argparse.ArgumentParser(
        description="""Load planning problems data saved to a YAML file, use it to build PlanningProblem objects, and
        use them to update the roadmap. The roadmap can also be supplemented with random sampling, with a number of
        samples specified by the user."""
    )
    parser.add_argument(
        "--num_problems",
        type=int,
        help="Number of saved planning problems to load, solve and update the roadmap",
        default=0,
    )
    parser.add_argument(
        "--num_samples", type=int, help="Number of samples to use in a random sampling update of the roadmap", default=0
    )
    parser.add_argument("--addr", type=str, help="Address of the IRIS builder service", default="localhost:5150")
    parser.add_argument("--context_id", type=int, help="ID of the context", required=True)
    parser.add_argument(
        "--num_fpp_problems", type=int, help="Number of FPP problems to load, solve and update the roadmap", default=0
    )
    parser.add_argument(
        "--num_random_ik_seed_samples",
        type=int,
        help="Number of random IK seed samples to load, solve and update the roadmap",
        default=0,
    )

    args = parser.parse_args()

    # Create iris client
    iris_client = IrisBuilderClient(addr=args.addr)
    iris_client.set_context_by_id("spoon_scene", args.context_id)

    print("Updating roadmap...")
    if args.num_samples > 0:
        print(f"Number of config space samples specified: {args.num_samples}")
        iris_client.update_roadmap_with_samples(args.num_samples)
    # Update the roadmap with the loaded planning problems
    print(f"Number of problems specified: {args.num_problems}")
    print(f"Number of FPP problems specified: {args.num_fpp_problems}")
    print(f"Number of random IK seed samples specified: {args.num_random_ik_seed_samples}")
    iris_client.update_roadmap_from_saved_problems(
        args.num_problems, args.num_fpp_problems, args.num_random_ik_seed_samples
    )


if __name__ == "__main__":
    main()
