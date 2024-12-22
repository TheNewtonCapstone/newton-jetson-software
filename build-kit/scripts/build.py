import os
import sys
import subprocess
import pprint
import argparse
import typing

import click

@click.command()
@click.option("--count", default=1, help="Number of greetings.")
@click.option("--name", prompt="Your name", help="The person to greet.")
def hello(count, name):
    """Simple program that greets NAME for a total of COUNT times."""
    for _ in range(count):
        click.echo(f"Hello, {name}!")

if __name__ == '__main__':
    hello()
# parser = argparse.ArgumentParser()


# parser.add_argument('packages', type=str, nargs='*', default=[],
#                     help='packages or containers to build')
# # TODO - The idea is to make the build system platform-agnostic, to be implemented
# parser.add_argument('--arch', type=str, default='arm64',)
# parser.add_argument('--name', type=str, default='', help="the name of the output container to build")
# parser.add_argument('--base', type=str, default='',
#                     help="the base container to use at the beginning of the build chain (default: l4t-jetpack)")
# # TODO - add support for multiple base containers, for now we are only doing chaining.
# parser.add_argument('--multiple', action='store_true',
#                     help="the specified packages should be built independently as opposed to chained together")
# parser.add_argument('--package-dirs', type=str, default='',
#                     help="additional package search directories (comma or colon-separated)")
#
#
# # TODO - list packages
# parser.add_argument('--list-packages', action='store_true',
#                     help="show the list of packages that were found under the search directories")
# # TODO - show packages
# parser.add_argument('--show-packages', action='store_true',
#                     help="show info about one or more packages (if none are specified, all will be listed")
# parser.add_argument('--verbose', action='store_true', help="show verbose output (for debugging)")
#
# args = parser.parse_args()
#
# if __name__ == '__main__':
#     if args.verbose:
#         os.environ['VERBOSE'] = 'ON'