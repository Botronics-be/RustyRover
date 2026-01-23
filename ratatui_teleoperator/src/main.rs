use std::{ops::Div, time::{Duration, Instant}};

use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind};
use ratatui::{
    buffer::Buffer,
    layout::{Rect, Constraint, Direction, Layout},
    style::{Color, Stylize, Style},
    symbols::border,
    text::{Line, Text, Span},
    widgets::{Block, Paragraph, Widget},
    Frame,
};
use color_eyre::{
    Result, eyre::{Ok, WrapErr}
};
mod tui;

#[derive(Debug)]
pub struct App {
    linear_command: i8,
    angular_command: i8,
    last_linear_cmd_stamp: Instant,
    last_angular_cmd_stamp: Instant,
    exit: bool,
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let mut terminal = tui::init()?;
    let mut app = App {
        linear_command: 0,
        angular_command: 0,
        last_linear_cmd_stamp: Instant::now(),
        last_angular_cmd_stamp: Instant::now(),
        exit: false
    };
    let app_result = app.run(&mut terminal);
    if let Err(err) = tui::restore() {
        eprintln!(
            "failed to restore terminal. Run `reset` or restart your terminal to recover: {err}"
        );
    }
    app_result
}


impl App {

    pub fn run(&mut self, terminal: &mut tui::Tui) -> Result<()> {
        while !self.exit {
            terminal.draw(|frame| self.render_frame(frame))?;
            if event::poll(Duration::from_millis(16))? {
                self.handle_events().wrap_err("handle events failed")?;
            }

            if self.linear_command != 0 && self.last_linear_cmd_stamp.elapsed() > Duration::from_millis(300) {
                self.reset_linear()?;
            }

            if self.angular_command != 0 && self.last_angular_cmd_stamp.elapsed() > Duration::from_millis(300) {
                self.reset_angular()?;
            }
        }
        Ok(())
    }

    fn render_frame(&self, frame: &mut Frame) {
        frame.render_widget(self, frame.area());
    }

    fn handle_events(&mut self) -> Result<()> {
        match event::read()? {
            Event::Key(key_event) if key_event.kind == KeyEventKind::Press => self
                .handle_key_event(key_event)
                .wrap_err_with(|| format!("handling key event failed:\n{key_event:#?}")),
            _ => Ok(()),
        }
    }

    fn handle_key_event(&mut self, key_event: KeyEvent) -> Result<()>{
        if key_event.is_press() {
            match key_event.code {
                KeyCode::Char('q') => self.exit()?,
                KeyCode::Char(' ') => self.stop_robot()?,
                KeyCode::Up => self.increment_linear()?,
                KeyCode::Down => self.decrement_linear()?,
                KeyCode::Left => self.increment_angular()?,
                KeyCode::Right => self.decrement_angular()?,
                _ => {}
            }
        } else if key_event.is_release() {
            match key_event.code {
                KeyCode::Up => self.reset_linear()?,
                KeyCode::Down => self.reset_linear()?,
                KeyCode::Left => self.reset_angular()?,
                KeyCode::Right => self.reset_angular()?,
                _ => {}
            }
        }
        
        Ok(())
    }

    fn stop_robot(&mut self) -> Result<()> {
        self.linear_command = 0;
        self.angular_command = 0;
        Ok(())
    }

    fn increment_linear(&mut self) -> Result<()> {
        if self.linear_command < 0 {
            self.linear_command = (self.linear_command + 15).clamp(-100, 100);
        } else {
            self.linear_command = (self.linear_command + 5).clamp(-100, 100);
        }
        self.last_linear_cmd_stamp = Instant::now();
        Ok(())
    }

    fn decrement_linear(&mut self) -> Result<()> {        
        if self.linear_command > 0 {
            self.linear_command = (self.linear_command - 15).clamp(-100, 100);
        } else {
            self.linear_command = (self.linear_command - 5).clamp(-100, 100);
        }
        self.last_linear_cmd_stamp = Instant::now();
        Ok(())
    }


    fn increment_angular(&mut self) -> Result<()> {
        if self.angular_command < 0 {
            self.angular_command = (self.angular_command + 25).clamp(-100, 100);
        } else {
            self.angular_command = (self.angular_command + 10).clamp(-100, 100);
        }
        self.last_angular_cmd_stamp = Instant::now();
        Ok(())
    }


    fn decrement_angular(&mut self) -> Result<()> {
        if self.angular_command > 0 {
            self.angular_command = (self.angular_command - 25).clamp(-100, 100);
        } else {
            self.angular_command = (self.angular_command - 10).clamp(-100, 100);
        }
        
        self.last_angular_cmd_stamp = Instant::now();
        Ok(())
    }

    fn reset_linear(&mut self) -> Result<()> {
        if self.linear_command > 0 {
            self.linear_command = (self.linear_command - 5).clamp(-100, 100);
        } else {
            self.linear_command = (self.linear_command + 5).clamp(-100, 100);
        }
        Ok(())
    }

    fn reset_angular(&mut self) -> Result<()> {
        if self.angular_command > 0 {
            self.angular_command = (self.angular_command - 5).clamp(-100, 100);
        } else {
            self.angular_command = (self.angular_command + 5).clamp(-100, 100);
        }
        Ok(())
    }

    fn exit(&mut self) -> Result<()>{
        self.exit = true;
        Ok(())
    }
}

impl Widget for &App {
    fn render(self, area: Rect, buf: &mut Buffer) {
        // 1. Define the Layout
        // Split the screen: Top part for the Crab, bottom part (3 lines) for the status/controls
        let layout = Layout::default()
            .direction(Direction::Vertical)
            .constraints(vec![
                Constraint::Min(10),   // Give the crab at least 10 lines, or the rest of the screen
                Constraint::Length(12), // Space for your instructions block
            ])
            .split(area);

        // 2. Load and Style the ASCII Art
        // include_str! compiles the file directly into your binary
        let raw_art = include_str!("../crab_1.txt"); 
        
        let art_lines: Vec<Line> = raw_art
            .lines()
            .filter(|line| !line.trim().starts_with("[source")) // Remove metadata lines if present
            .map(|line| {
                let spans: Vec<Span> = line
                    .chars()
                    .map(|c| {
                        if c.is_whitespace() {
                            // Return 2 spaces to match the width of the double-block below
                            // If you use single blocks, change this to " "
                            return Span::raw(" "); 
                        }

                        // 2. Map characters to colors
                        let color = match c {
                            '%' => Color::LightRed,
                            '#' => Color::DarkGray,
                            '$' => Color::White,
                            '@' => Color::Black,
                            _ => Color::Reset,
                        };

                        // 3. Render "Squares"
                        // We use two block characters "██" because terminal fonts are tall.
                        // This makes the output look like square pixels.
                        Span::styled("█", Style::default().fg(color))
                    })
                    .collect();
                Line::from(spans)
            })
            .collect();

        let art_widget = Paragraph::new(Text::from(art_lines))
            .centered() // Center the crab in the top box
            .block(Block::bordered().title(" Rusty Rover Camera Feed ").border_style(Style::default().fg(Color::DarkGray)));

        // 3. Render the Art in the top chunk
        art_widget.render(layout[0], buf);

        // 4. Render Your Existing Controls in the bottom chunk
        // (This is your original code, just pointing to layout[1] instead of area)
        let title = Line::from(" RustyRover Teleoperator ".bold());
        let instructions = Line::from(vec![
            " Stop ".into(),
            "<Space>".light_red().bold(),
            " Accelerate ".into(),
            "<Up>".blue().bold(),
            " Decelerate ".into(),
            "<Down>".blue().bold(),
            " Turn Left ".into(),
            "<Left>".blue().bold(),
            " Turn Right ".into(),
            "<Right>".blue().bold(),            
            " Quit ".into(),
            "<Q> ".blue().bold(),
        ]);
        
        let block = Block::bordered()
            .title(title.centered())
            .title_bottom(instructions.centered())
            .border_set(border::THICK);

        let counter_text = Text::from(vec![
            Line::from(""), // Add some padding
            Line::from(vec![
                "Linear command: ".into(),
                (self.linear_command as f32).div(10.0).to_string().yellow(),
                "  Angular command: ".into(),
                (self.angular_command as f32).div(10.0).to_string().yellow(),            
            ])
        ]);

        Paragraph::new(counter_text)
            .centered()
            .block(block)
            .render(layout[1], buf);
    }
}