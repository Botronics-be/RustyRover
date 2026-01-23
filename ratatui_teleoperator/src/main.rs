use std::{ops::Div, time::{Duration, Instant}};

use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind};
use ratatui::{
    buffer::Buffer,
    layout::Rect,
    style::Stylize,
    symbols::border,
    text::{Line, Text},
    widgets::{Block, Paragraph, Widget},
    Frame,
};
use color_eyre::{
    Result, eyre::{Ok, WrapErr}
};

#[derive(Debug)]
pub struct App {
    linear_command: i8,
    angular_command: i8,
    last_linear_cmd_stamp: Instant,
    last_angular_cmd_stamp: Instant,
    exit: bool,
}

mod tui;

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

            if self.linear_command != 0 && self.last_linear_cmd_stamp.elapsed() > Duration::from_millis(500) {
                self.reset_linear()?;
            }

            if self.angular_command != 0 && self.last_angular_cmd_stamp.elapsed() > Duration::from_millis(500) {
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
        self.linear_command = 0;
        Ok(())
    }

    fn reset_angular(&mut self) -> Result<()> {
        self.angular_command = 0;
        Ok(())
    }

    fn exit(&mut self) -> Result<()>{
        self.exit = true;
        Ok(())
    }
}

impl Widget for &App {
    fn render(self, area: Rect, buf: &mut Buffer) {
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

        let counter_text = Text::from(vec![Line::from(vec![
            "Linear command: ".into(),
            (self.linear_command as f32).div(10.0).to_string().yellow(),
            "  Angular command: ".into(),
            (self.angular_command as f32).div(10.0).to_string().yellow(),            
        ])]);

        Paragraph::new(counter_text)
            .centered()
            .block(block)
            .render(area, buf);
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use ratatui::style::Style;

//     #[test]
//     fn render() {
//         let app = App::default();
//         let mut buf = Buffer::empty(Rect::new(0, 0, 50, 4));

//         app.render(buf.area, &mut buf);

//         let mut expected = Buffer::with_lines(vec![
//             "┏━━━━━━━━━━━━━ Counter App Tutorial ━━━━━━━━━━━━━┓",
//             "┃                    Value: 0                    ┃",
//             "┃                                                ┃",
//             "┗━ Decrement <Left> Increment <Right> Quit <Q> ━━┛",
//         ]);
//         let title_style = Style::new().bold();
//         let counter_style = Style::new().yellow();
//         let key_style = Style::new().blue().bold();
//         expected.set_style(Rect::new(14, 0, 22, 1), title_style);
//         expected.set_style(Rect::new(28, 1, 1, 1), counter_style);
//         expected.set_style(Rect::new(13, 3, 6, 1), key_style);
//         expected.set_style(Rect::new(30, 3, 7, 1), key_style);
//         expected.set_style(Rect::new(43, 3, 4, 1), key_style);

//         assert_eq!(buf, expected);
//     }

//     #[test]
//     fn handle_key_event() {
//         let mut app = App::default();
//         app.handle_key_event(KeyCode::Right.into()).unwrap();
//         assert_eq!(app.counter, 1);

//         app.handle_key_event(KeyCode::Left.into()).unwrap();
//         assert_eq!(app.counter, 0);

//         let mut app = App::default();
//         app.handle_key_event(KeyCode::Char('q').into()).unwrap();
//         assert!(app.exit);
//     }

//     #[test]
//     #[should_panic(expected = "attempt to subtract with overflow")]
//     fn handle_key_event_panic() {
//         let mut app = App::default();
//         let _ = app.handle_key_event(KeyCode::Left.into());
//     }

//     #[test]
//     fn handle_key_event_overflow() {
//         let mut app = App::default();
//         assert!(app.handle_key_event(KeyCode::Right.into()).is_ok());
//         assert!(app.handle_key_event(KeyCode::Right.into()).is_ok());
//         assert_eq!(
//             app.handle_key_event(KeyCode::Right.into())
//                 .unwrap_err()
//                 .to_string(),
//             "counter overflow"
//         );
//     }
// }