"""
Entry point for running nimbus as a module.

Usage: python -m nimbus [command]
"""

from nimbus.cli.app import app

if __name__ == "__main__":
    app()
