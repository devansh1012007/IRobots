# orchestrator.py

import os
import sys
import json
import time
from rich.console import Console
from rich.panel import Panel
from rich.text import Text
from rich.live import Live
from rich.layout import Layout
from rich.columns import Columns
from rich.table import Table
from rich.markdown import Markdown

# Import Project Modular Nodes
import config
from get_image import capture_and_save_frame
from sam2_processor import SAM2Processor
from agent_harness import AgentHarness

console = Console()

class RoboticOrchestrator:
    def __init__(self, offline_mode: bool = False):
        self.offline_mode = offline_mode
        self.console = console
        
        # Initialize Core Nodes
        self.sam_processor = SAM2Processor(config=config.sam2_processor_CONFIG)
        self.agent_harness = AgentHarness(cfg=config.agent_harness_CONFIG, tools=config.ROBOT_TOOLS)

    def print_banner(self):
        """Renders OpenCode-style ASCII / Rich TUI Banner."""
        self.console.clear()
        banner_text = Text()
        banner_text.append("   ____  ____  ______  _  _______  ____  _______\n", style="bold cyan")
        banner_text.append("  / __ \\/ __ \\/ __/ / / / / ___/ / __ \\/ __/ _ \\\n", style="bold cyan")
        banner_text.append(" / /_/ / /_/ / _// /_/ / / /__/ / /_/ / _// ___/\n", style="bold blue")
        banner_text.append(" \\____/ .___/___/\\____/  \\___/  \\____/___/_/    \n", style="bold blue")
        banner_text.append("     /_/   ROBOTIC VISION & ACTUATION SYSTEM v2.1\n", style="dim white")

        self.console.print(Panel(banner_text, border_style="bold cyan", expand=False))
        
        # System Info Bar
        info_table = Table(show_header=False, box=None, padding=(0, 2))
        info_table.add_row(
            f"[bold magenta]LM Studio:[/bold magenta] {config.agent_harness_CONFIG['LM_STUDIO_URL']}",
            f"[bold yellow]Model:[/bold yellow] {config.agent_harness_CONFIG['MODEL_NAME']}",
            f"[bold green]Mode:[/bold green] {'OFFLINE (Sample File)' if self.offline_mode else 'ONLINE (ESP32-CAM)'}"
        )
        self.console.print(info_table)
        self.console.print("[dim]─" * 80 + "[/dim]\n")

    def run_pipeline_step(self, user_command: str):
        """Executes a single atomic loop turn across all 3 nodes."""
        
        # =========================================================================
        # STEP 1: IMAGE CAPTURE (Camera Node)
        # =========================================================================
        raw_image_path = "raw_capture.jpg"
        
        with self.console.status("[bold green]Step 1/3: Acquiring Frame from ESP32-CAM...", spinner="dots"):
            if self.offline_mode or not os.path.exists("test_artifacts"):
                # Fallback to local test image if offline or hardware not connected
                raw_image_path = config.agent_harness_CONFIG.get("TEST_IMAGE_PATH", "test_artifacts/test_image6.jpg")
                time.sleep(0.5)
            else:
                try:
                    raw_image_path = capture_and_save_frame(output_filename="raw_capture.jpg")
                except Exception as e:
                    self.console.print(f"[bold red]❌ Camera Fetch Failed ({e}). Falling back to test sample...[/bold red]")
                    raw_image_path = config.agent_harness_CONFIG["TEST_IMAGE_PATH"]

        self.console.print(f"  [bold check]✓[/bold check] Image Acquired: [dim]{raw_image_path}[/dim]")

        # =========================================================================
        # STEP 2: SAM 2.1 SEGMENTATION & NOISE FILTERING
        # =========================================================================
        with self.console.status("[bold yellow]Step 2/3: Segmenting & Cleaning Noise via SAM 2.1...", spinner="dots"):
            _, detected_objects = self.sam_processor.process_image(image_input=raw_image_path)

        # Render SAM Metadata Table to TUI
        sam_table = Table(title="SAM 2.1 Detected Objects", show_header=True, header_style="bold magenta")
        sam_table.add_column("ID", style="cyan", width=6)
        sam_table.add_column("Center (X, Y) cm", style="green")
        sam_table.add_column("Center (X, Y) px", style="yellow")
        sam_table.add_column("Bounding Box [x1, y1, x2, y2]", style="dim")

        for obj in detected_objects:
            sam_table.add_row(
                str(obj["id"]),
                f"({obj['centroid_cm']['x']}, {obj['centroid_cm']['y']})",
                f"({obj['centroid_px']['x']}, {obj['centroid_px']['y']})",
                str(obj["bounding_box_pixels"])
            )

        self.console.print(Panel(sam_table, border_style="yellow", title="[bold yellow]Perception Layer[/bold yellow]"))

        # =========================================================================
        # STEP 3: AGENT REASONING & STREAMING TOOL CALL
        # =========================================================================
        self.console.print(Panel(f"[bold white]User Command:[/bold white] {user_command}", border_style="blue"))
        
        self.console.print("\n[bold cyan]🧠 Model Thinking & Execution Stream:[/bold cyan]")
        
        # Execute streaming agent harness
        output_payload = self.agent_harness.decide_action(
            user_command=user_command,
            image_path=config.sam2_processor_CONFIG["OUTPUT_IMAGE"],
            detected_objects=detected_objects
        )

        # =========================================================================
        # STEP 4: API TOOL CALL OUTPUT DISPLAY
        # =========================================================================
        tool_data = output_payload.get("tool_call", {})
        feedback = output_payload.get("execution_feedback", {})

        status_color = "green" if feedback.get("status") == "success" else "red" if feedback.get("status") == "error" else "yellow"

        tool_summary = Text()
        tool_summary.append("Tool Executed : ", style="bold white")
        tool_summary.append(f"{tool_data.get('tool')}\n", style="bold cyan")
        tool_summary.append("Arguments     : ", style="bold white")
        tool_summary.append(f"{json.dumps(tool_data.get('arguments'))}\n", style="yellow")
        tool_summary.append("API Feedback  : ", style="bold white")
        tool_summary.append(f"[{feedback.get('status', 'info').upper()}] {feedback.get('message')}", style=f"bold {status_color}")

        self.console.print(Panel(tool_summary, title="[bold green]🤖 Hardware API Dispatch Output[/bold green]", border_style=status_color))

        # TARGETED FIX: Return True if a tool call was executed, False if text-only response
        executed_tool = tool_data.get("tool")
        return executed_tool is not None and executed_tool != "text_response"

    def start_chat_loop(self):
        """Interactive Terminal Loop."""
        self.print_banner()
        
        while True:
            try:
                user_input = self.console.input("\n[bold cyan]iRobots > [/bold cyan]").strip()
                
                if not user_input:
                    continue
                
                if user_input.lower() in ["exit", "quit", "q", "/exit"]:
                    self.console.print("[bold red]Shutting down orchestrator... Goodbye![/bold red]")
                    break
                
                if user_input.lower() in ["/clear", "clear"]:
                    self.print_banner()
                    continue

                step_command = user_input
                max_auto_turns = 10  # Guardrail against infinite loops
                turn_count = 0

                while turn_count < max_auto_turns:
                    tool_was_executed = self.run_pipeline_step(user_command=step_command)
                    
                    if not tool_was_executed:
                        # Model finished tasks or replied with pure text; yield back to user
                        break

                    turn_count += 1
                    self.console.print(f"\n[bold yellow]🔄 Auto-triggering Next Visual Feedback Loop (Turn {turn_count + 1})...[/bold yellow]\n")
                    
                    # Subsequent autonomous turns use an internal instruction context
                    step_command = "Assess the updated camera view and execute the next atomic step to complete the task."
                
                if turn_count >= max_auto_turns:
                    self.console.print("[bold red]⚠️ Safety Limit: Reached maximum autonomous turn limit.[/bold red]")

            except KeyboardInterrupt:
                self.console.print("\n[bold red]Interrupted by user. Exiting...[/bold red]")
                sys.exit(0)


# ==============================================================================
# 🚀 ORCHESTRATOR ENTRYPOINT
# ==============================================================================
if __name__ == "__main__":
    # Check for --offline argument to test without ESP32 connected
    is_offline = "--offline" in sys.argv
    
    # Check if rich library is present
    try:
        orchestrator = RoboticOrchestrator(offline_mode=is_offline)
        orchestrator.start_chat_loop()
    except ModuleNotFoundError:
        print("Error: 'rich' library is required for the OpenCode TUI interface.")
        print("Install it inside WSL using: pip install rich")