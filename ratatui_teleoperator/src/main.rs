mod tui;
mod ble_client;
use ble_client::{RobotCommand, ToBle, FromBle};
use tokio::sync::mpsc;
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
    Result, eyre::{WrapErr}
};


#[derive(Debug)]
pub struct App {
    linear_command: i8,
    angular_command: i8,
    last_linear_cmd_stamp: Instant,
    last_angular_cmd_stamp: Instant,
    ble_tx: mpsc::UnboundedSender<ToBle>,
    ble_rx: mpsc::UnboundedReceiver<FromBle>,
    ble_status: String,
    exit: bool,
}

#[tokio::main]
async fn main() -> Result<()> {
    color_eyre::install()?;
    let mut terminal = tui::init()?;

    let (to_ble_tx, to_ble_rx) = mpsc::unbounded_channel();
    let (from_ble_tx, from_ble_rx) = mpsc::unbounded_channel();

    tokio::spawn(async move {
        ble_client::run_ble_worker(to_ble_rx, from_ble_tx).await;
    });

    let mut app = App {
        exit: false,

        linear_command: 0,
        angular_command: 0,
        last_linear_cmd_stamp: Instant::now(),
        last_angular_cmd_stamp: Instant::now(),

        ble_tx: to_ble_tx,
        ble_rx: from_ble_rx,
        ble_status: "Disconnected".to_string(),        
    };

    let app_result = app.run(&mut terminal).await;

    if let Err(err) = tui::restore() {
        eprintln!("failed to restore terminal: {err}");
    }
    app_result
}


impl App {

    pub async fn run(&mut self, terminal: &mut tui::Tui) -> Result<()> {
        while !self.exit {
            terminal.draw(|frame| self.render_frame(frame))?;

            while let Ok(msg) = self.ble_rx.try_recv() {
                match msg {
                    FromBle::StatusChange(s) => self.ble_status = s,
                    FromBle::DataReceived(d) => self.ble_status = format!("Data: {}", d),
                }
            }

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
                KeyCode::Char('c') => self.connect_server()?,
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

    fn connect_server(&mut self) -> Result<()> {
        match self.ble_tx.send(ToBle::Connect){
            Ok(()) => Ok(()),
            Err(e) =>{
                eprintln!("Error while connecting : {}", e);
                Err(e.into())
            }
        }
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

        let data = self.format_teleop_cmd();
        return self.send_command("TELEOP".to_string(), data);
    }

    fn decrement_linear(&mut self) -> Result<()> {        
        if self.linear_command > 0 {
            self.linear_command = (self.linear_command - 15).clamp(-100, 100);
        } else {
            self.linear_command = (self.linear_command - 5).clamp(-100, 100);
        }

        self.last_linear_cmd_stamp = Instant::now();

        let data = self.format_teleop_cmd();
        return self.send_command("TELEOP".to_string(), data);
    }


    fn increment_angular(&mut self) -> Result<()> {
        if self.angular_command < 0 {
            self.angular_command = (self.angular_command + 25).clamp(-100, 100);
        } else {
            self.angular_command = (self.angular_command + 10).clamp(-100, 100);
        }

        self.last_angular_cmd_stamp = Instant::now();

        let data = self.format_teleop_cmd();
        return self.send_command("TELEOP".to_string(), data);
    }


    fn decrement_angular(&mut self) -> Result<()> {
        if self.angular_command > 0 {
            self.angular_command = (self.angular_command - 25).clamp(-100, 100);
        } else {
            self.angular_command = (self.angular_command - 10).clamp(-100, 100);
        }
        
        self.last_angular_cmd_stamp = Instant::now();
        let data = self.format_teleop_cmd();

        return self.send_command("TELEOP".to_string(), data);
    }

    fn reset_linear(&mut self) -> Result<()> {
        if self.linear_command > 0 {
            self.linear_command = (self.linear_command - 5).clamp(-100, 100);
        } else {
            self.linear_command = (self.linear_command + 5).clamp(-100, 100);
        }

        let data = self.format_teleop_cmd();
        return self.send_command("TELEOP".to_string(), data);
    }

    fn reset_angular(&mut self) -> Result<()> {
        if self.angular_command > 0 {
            self.angular_command = (self.angular_command - 5).clamp(-100, 100);
        } else {
            self.angular_command = (self.angular_command + 5).clamp(-100, 100);
        }

        let data = self.format_teleop_cmd();
        return self.send_command("TELEOP".to_string(), data);
    }

    fn exit(&mut self) -> Result<()>{
        self.exit = true;
        Ok(())
    }

    fn format_teleop_cmd(&mut self) -> String {
        format!("{{\"linear_x\": {}, \"angular_z\": {}}}", (self.linear_command as f64).div(10.0), (self.angular_command as f64).div(10.0))
    }

    fn send_command(&mut self, cmd: String, data: String) -> Result<()>{
        let ble_msg = RobotCommand {
            cmd_type: cmd,
            data: data,
        };
        let _ = self.ble_tx.send(ToBle::SendJson(ble_msg));
        Ok(())
    }
}

impl Widget for &App {
    fn render(self, area: Rect, buf: &mut Buffer) {
        let layout = Layout::default()
            .direction(Direction::Vertical)
            .constraints(vec![
                Constraint::Min(50),
                Constraint::Length(48),
            ])
            .split(area);

        let raw_art: &str;
        
        if self.last_linear_cmd_stamp.elapsed().as_secs() % 2 == 0 {
            raw_art = include_str!("../ascii_art/crab_1.txt"); 
        } else {
            raw_art = include_str!("../ascii_art/crab_0.txt"); 
        };
        
        let art_lines: Vec<Line> = raw_art
            .lines()
            .filter(|line| !line.trim().starts_with("[source")) // Remove metadata lines if present
            .map(|line| {
                let spans: Vec<Span> = line
                    .chars()
                    .map(|c| {
                        if c.is_whitespace() {
                            return Span::raw(" "); 
                        }

                        let color = match c {
                            '%' => Color::LightRed,
                            '#' => Color::DarkGray,
                            '$' => Color::White,
                            '@' => Color::Black,
                            _ => Color::Reset,
                        };

                        Span::styled("█", Style::default().fg(color))
                    })
                    .collect();
                Line::from(spans)
            })
            .collect();

        let art_widget = Paragraph::new(Text::from(art_lines))
            .centered()
            .block(Block::bordered().border_style(Style::default().fg(Color::White)));

        art_widget.render(layout[0], buf);

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
            " Connect ".into(),
            "<C> ".blue().bold(),
        ]);
        
        let block = Block::bordered()
            .title(title.centered())
            .title_bottom(instructions.centered())
            .border_set(border::THICK);

        let status_text = Text::from(vec![
            Line::from(""),
            Line::from(vec![
                "  Linear command:    ".into(),
                (self.linear_command as f32).div(10.0).to_string().yellow(),            
            ]),
            Line::from(vec![
                "  Angular command:   ".into(),
                (self.angular_command as f32).div(10.0).to_string().yellow(),            
            ]),
            Line::from(vec![
                "  Bluetooth status:  ".into(),
                self.ble_status.to_string().yellow(), 
            ]),
        ]);

        Paragraph::new(status_text)
            .block(block)
            .render(layout[0], buf);
    }

}